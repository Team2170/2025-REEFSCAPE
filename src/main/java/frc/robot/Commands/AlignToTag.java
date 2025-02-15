// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import BobcatLib.Subsystems.Swerve.SimpleSwerve.SwerveDrive;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants.AutoAlignmentConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToTag extends Command {
  SwerveDrive s_Swerve;
  ProfiledPIDController angleController;
  ProfiledPIDController xController;
  ProfiledPIDController yController;
  LinearFilter xFilter;
  LinearFilter yFilter;
  Supplier<Rotation2d> tx;
  DoubleSupplier ty;
  DoubleSupplier distanceToTag;

  /** Creates a new AlignToTag. */
  public AlignToTag(SwerveDrive drive, Supplier<Rotation2d> tx, DoubleSupplier ty, DoubleSupplier distanceToTag) {
    this.s_Swerve = drive;
    this.tx = tx;
    this.ty = ty;
    this.distanceToTag = distanceToTag;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.s_Swerve);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Create PID controller
    angleController = new ProfiledPIDController(
        AutoAlignmentConstants.ANGLE_KP,
        0.0,
        AutoAlignmentConstants.ANGLE_KD,
        new TrapezoidProfile.Constraints(AutoAlignmentConstants.ANGLE_MAX_VELOCITY,
            AutoAlignmentConstants.ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    xController = new ProfiledPIDController(
        AutoAlignmentConstants.DRIVE_KPX, 0.0, AutoAlignmentConstants.DRIVE_KDX,
        new TrapezoidProfile.Constraints(1, 1.0));
    xFilter = LinearFilter.movingAverage(50);

    yController = new ProfiledPIDController(
        AutoAlignmentConstants.DRIVE_KPY, 0.0, AutoAlignmentConstants.DRIVE_KDY,
        new TrapezoidProfile.Constraints(1, 1.0));
    yFilter = LinearFilter.movingAverage(50);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distToTag = distanceToTag.getAsDouble();
    double yDist = ty.getAsDouble();
    double filteredDistance = 0;
    double filteredY = 0;

    if (distToTag != 0) {
      filteredDistance = xFilter.calculate(distToTag);
    }
    if (ty.getAsDouble() != 0) {
      filteredY = yFilter.calculate(yDist);
    }

    Logger.recordOutput("FilteredX", filteredDistance);
    Logger.recordOutput("FilteredY", filteredY);

    Logger.recordOutput("DistToTag", filteredDistance);
    // Calculate angular speed
    double yawInRadians = s_Swerve.getGyroYaw().getRadians();
    double omega = tx.get().getRadians() == 0
        ? 0
        : angleController.calculate(
          yawInRadians, tx.get().getRadians());

    double distanceOutput = distanceToTag.getAsDouble() == 0 ? 0 : xController.calculate(distToTag, 2);
    Logger.recordOutput("disttotagpid", distanceOutput);

    double yOutput = yController.calculate(yDist, 0);

    // Convert to field relative speeds & send command
    ChassisSpeeds speeds = new ChassisSpeeds(
        omega == 0
            ? distanceOutput * (1 / omega)
            : distanceOutput, // get to within 1 meter of the tag, output scales as
        // angular error decreases\
        yOutput,
        0);
    boolean isFlipped = DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;

    Rotation2d unadjustedYaw = s_Swerve.getGyroYaw();
    Rotation2d adjustedYaw = unadjustedYaw.plus(new Rotation2d(Math.PI));
    ChassisSpeeds rawSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      speeds,
      isFlipped
          ? adjustedYaw
          : unadjustedYaw);

    s_Swerve.drive(rawSpeeds);
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
}
