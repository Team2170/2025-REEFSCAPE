// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import BobcatLib.Subsystems.Swerve.SimpleSwerve.SwerveDrive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignOnReef extends Command {
  private SwerveDrive s_Swerve;

  List<Pose2d> faces;

  ProfiledPIDController angleController;

  ProfiledPIDController xController;
  ProfiledPIDController yController;
  DoubleSupplier xSupplier;
  DoubleSupplier ySupplier;
  DoubleSupplier rotationSupplier;
  BooleanSupplier clockwiseSupplier;
  BooleanSupplier counterclockwiseSupplier;

  /** Creates a new SingleTagAlign. */
  public AlignOnReef(SwerveDrive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier clockwiseSupplier,
      BooleanSupplier counterclockwiseSupplier) {

    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.rotationSupplier = rotationSupplier;
    this.clockwiseSupplier = clockwiseSupplier;
    this.counterclockwiseSupplier = counterclockwiseSupplier;

    this.s_Swerve = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.s_Swerve);

    faces = Arrays.asList(FieldConstants.Reef.centerFaces);

    angleController = new ProfiledPIDController(
        Constants.AutoAlignmentConstants.ANGLE_KP,
        0.0,
        Constants.AutoAlignmentConstants.ANGLE_KD,
        new TrapezoidProfile.Constraints(Constants.AutoAlignmentConstants.ANGLE_MAX_VELOCITY,
            Constants.AutoAlignmentConstants.ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    xController = new ProfiledPIDController(
        Constants.AutoAlignmentConstants.DRIVE_KPX, 0.0, Constants.AutoAlignmentConstants.DRIVE_KDX,
        new TrapezoidProfile.Constraints(5, 3.0));

    yController = new ProfiledPIDController(
        Constants.AutoAlignmentConstants.DRIVE_KPY, 0.0, Constants.AutoAlignmentConstants.DRIVE_KDY,
        new TrapezoidProfile.Constraints(5, 3.0));

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.reset(s_Swerve.getPose().getX());
    yController.reset(s_Swerve.getPose().getY());
    angleController.reset(s_Swerve.getPose().getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Pose2d nearestFace = s_Swerve.getPose().nearest(faces);
    Logger.recordOutput("reef_face/raw", nearestFace);
    double adjustY = 0;

    if (clockwiseSupplier.getAsBoolean()) {
      adjustY = -FieldConstants.Reef.reefToBranchY;
    } else if (counterclockwiseSupplier.getAsBoolean()) {
      adjustY = FieldConstants.Reef.reefToBranchY;
    }

    int faceIndex = -1;
    for (int i = 0; i < FieldConstants.Reef.centerFaces.length; i++) {
      if (FieldConstants.Reef.centerFaces[i] == nearestFace) {
        faceIndex = i;
        break;
      }
    }

    Pose2d poseDirection = new Pose2d(
        FieldConstants.Reef.center, Rotation2d.fromDegrees(180 - (60 * faceIndex)));

    double adjustX = Constants.AutoAlignmentConstants.ALIGN_DISTANCE.baseUnitMagnitude()
        + FieldConstants.Reef.faceToCenter;
    // double adjustY = Units.inchesToMeters(0);

    Pose2d offsetFace = new Pose2d(
        new Translation2d(
            poseDirection
                .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                .getX(),
            poseDirection
                .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                .getY()),
        new Rotation2d(poseDirection.getRotation().getRadians()));

    Logger.recordOutput("reef_face/offset", offsetFace);

    double yOutput = yController.calculate(s_Swerve.getPose().getY(), offsetFace.getY());
    double xOutput = xController.calculate(s_Swerve.getPose().getX(), offsetFace.getX());
    double omegaOutput = angleController.calculate(
        s_Swerve.getPose().getRotation().getRadians(),
        offsetFace.getRotation().getRadians());

    Logger.recordOutput("driveToReef/xError", xController.getPositionError());
    Logger.recordOutput("driveToReef/xPID", xOutput);
    Logger.recordOutput("driveToReef/yError", yController.getPositionError());
    Logger.recordOutput("driveToReef/yPID", yOutput);
    Logger.recordOutput("driveToReef/omegaError", angleController.getPositionError());
    Logger.recordOutput("driveToReef/omegaPID", omegaOutput);

    // Get linear velocity
    Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

    // Apply rotation deadband
    double omegaOverride = MathUtil.applyDeadband(rotationSupplier.getAsDouble(),
        Constants.AutoAlignmentConstants.DEADBAND);

    // Square rotation value for more precise control
    omegaOverride = Math.copySign(omegaOverride * omegaOverride, omegaOverride);

    // Convert to field relative speeds & send command
    double maxChassisSpeed = 4.5;
    double maxAngularSpeed = 4.5 / Constants.AutoAlignmentConstants.DRIVE_BASE_RADIUS;
    ChassisSpeeds speeds = new ChassisSpeeds(
        (linearVelocity.getX() + xOutput) * maxChassisSpeed,
        (linearVelocity.getY() + yOutput) * maxChassisSpeed,
        (omegaOutput + omegaOverride) * maxAngularSpeed);
    boolean isFlipped = DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
    s_Swerve.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped
                ? s_Swerve.getGyroYaw().plus(new Rotation2d(Math.PI))
                : s_Swerve.getGyroYaw()));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), Constants.AutoAlignmentConstants.DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }
}
