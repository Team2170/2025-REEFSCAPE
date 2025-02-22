// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.ejml.sparse.csc.linsol.qr.LinearSolverQrLeftLooking_DSCC;
import org.littletonrobotics.junction.Logger;

import BobcatLib.Hardware.Controllers.Axis;
import BobcatLib.Hardware.Controllers.parser.ControllerJson;
import BobcatLib.Hardware.Sensors.SpatialSensor.Spatial;
import BobcatLib.Hardware.Sensors.SpatialSensor.SpatialTOF;
import BobcatLib.Hardware.Sensors.SpatialSensor.Components.CANRange;
import BobcatLib.Hardware.Sensors.SpatialSensor.Components.RangeSensor;
import BobcatLib.Hardware.Sensors.SpatialSensor.Components.SENS3006;
import BobcatLib.Hardware.Sensors.SpatialSensor.Components.SimTOF;
import BobcatLib.Hardware.Sensors.SpatialSensor.Utility.DistanceMode;
import BobcatLib.Hardware.Sensors.SpatialSensor.Utility.DistanceMode.modes;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.SwerveDrive;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToSurface extends Command {
  private SwerveDrive s_Swerve;
  ProfiledPIDController angleController;
  DoubleSupplier xSupplier;
  DoubleSupplier ySupplier;
  DoubleSupplier rotationSupplier;
  BooleanSupplier clockwiseSupplier;
  BooleanSupplier counterclockwiseSupplier;
  /** Configuration details parsed from a JSON controller configuration. */
  private ControllerJson controllerJson;
  private boolean isSim;

  private final Spatial distanceSensing;
  private SpatialTOF stof;

  /** Creates a new AlginToSurface. */
  public AlignToSurface(SwerveDrive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier clockwiseSupplier,
      BooleanSupplier counterclockwiseSupplier, ControllerJson controllerJson, boolean isSim) {

    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.rotationSupplier = rotationSupplier;
    this.clockwiseSupplier = clockwiseSupplier;
    this.counterclockwiseSupplier = counterclockwiseSupplier;
    this.controllerJson = controllerJson;
    this.isSim = isSim;

    this.s_Swerve = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.s_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
    angleController = new ProfiledPIDController(
        Constants.AutoAlignmentConstants.ANGLE_KP,
        0.0,
        Constants.AutoAlignmentConstants.ANGLE_KD,
        new TrapezoidProfile.Constraints(Constants.AutoAlignmentConstants.ANGLE_MAX_VELOCITY,
            Constants.AutoAlignmentConstants.ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    List<RangeSensor> distanceSensors = new ArrayList<RangeSensor>();
    if (this.isSim) {
      SimTOF stof = new SimTOF(1, new DistanceMode(modes.SHORT), 20);
      distanceSensors.add(stof);
      SimTOF stof1 = new SimTOF(2, new DistanceMode(modes.SHORT), 20);
      distanceSensors.add(stof1);
    } else {
      distanceSensors.add(new CANRange(1, new DistanceMode(modes.SHORT), 20));
      distanceSensors.add(new CANRange(2, new DistanceMode(modes.SHORT), 20));
    }
    stof = new SpatialTOF(distanceSensors);
    distanceSensing = new Spatial(stof);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleController.reset(s_Swerve.getPose().getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distanceSensing.periodic();
    s_Swerve.periodic();
    // Retrieve drive limits from the swerve configuration
    double maxSpeed = s_Swerve.jsonSwerve.moduleSpeedLimits.maxSpeed;
    double maxAngularVelocity = s_Swerve.jsonSwerve.moduleSpeedLimits.maxAngularVelocity;
    double sensorDistance = 10;
    double leftDistance = distanceSensing.getDistances().get("left");
    double rightDistance = distanceSensing.getDistances().get("right");
    double goal = sensorDistance / (leftDistance - rightDistance);
    double omegaOutput = angleController.calculate(
        s_Swerve.getGyroYaw().getDegrees(),
        goal);
    double adjustedOmegaOutput = omegaOutput * maxAngularVelocity;
    Logger.recordOutput("alignToSurface/omegaError", angleController.getPositionError());
    Logger.recordOutput("alignToSurface/omegaOutput", omegaOutput);
    Logger.recordOutput("alignToSurface/adjustedOmegaOutput", adjustedOmegaOutput);
    Axis translation = new Axis(xSupplier.getAsDouble(), controllerJson.single.deadband);
    Axis strafe = new Axis(ySupplier.getAsDouble(), controllerJson.single.deadband);
    s_Swerve.drive(
        new Translation2d(translation.getDeadband(), strafe.getDeadband()).times(maxSpeed),
        adjustedOmegaOutput,
        true,
        s_Swerve.getHeading(),
        s_Swerve.getPose());
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
