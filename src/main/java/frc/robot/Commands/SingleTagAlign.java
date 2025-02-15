// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import BobcatLib.Subsystems.Swerve.SimpleSwerve.SwerveDrive;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoAlignmentConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SingleTagAlign extends Command {
  SwerveDrive s_Swerve;
  DoubleSupplier distanceSupplier;
  DoubleSupplier horizontalSupplier;
  Supplier<Rotation2d> omegaSupplier;

  ProfiledPIDController distanceController;
  ProfiledPIDController angleController;
  ProfiledPIDController horizontalController;
  ProfiledPIDController xController;
  ProfiledPIDController yController;
  LinearFilter xFilter;
  LinearFilter yFilter;
  LinearFilter omegaFilter;

  /** Creates a new SingleTagAlign. */
  public SingleTagAlign(SwerveDrive drive, DoubleSupplier distanceSupplier,
      DoubleSupplier horizontalSupplier,
      Supplier<Rotation2d> omegaSupplier) {
    this.s_Swerve = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.s_Swerve);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    angleController = new ProfiledPIDController(
        AutoAlignmentConstants.ANGLE_KP,
        0.0,
        AutoAlignmentConstants.ANGLE_KD,
        new TrapezoidProfile.Constraints(AutoAlignmentConstants.ANGLE_MAX_VELOCITY,
            AutoAlignmentConstants.ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    omegaFilter = LinearFilter.movingAverage(50);

    distanceController = new ProfiledPIDController(
        AutoAlignmentConstants.DRIVE_KPX, 0.0, AutoAlignmentConstants.DRIVE_KDX,
        new TrapezoidProfile.Constraints(1, 1.0));
    xFilter = LinearFilter.movingAverage(50);

    horizontalController = new ProfiledPIDController(
        AutoAlignmentConstants.DRIVE_KPY, 0.0, AutoAlignmentConstants.DRIVE_KDY,
        new TrapezoidProfile.Constraints(1, 1.0));
    yFilter = LinearFilter.movingAverage(50);

    angleController.reset(s_Swerve.getGyroYaw().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double distToTag = distanceSupplier.getAsDouble();
    double horizontalDistance = horizontalSupplier.getAsDouble();
    double omega = omegaSupplier.get().getRadians();
    double filteredDistance = 0;
    double filteredHorizontal = 0;
    double filteredOmega = 0;

    if (distToTag != 0) {
      filteredDistance = xFilter.calculate(distToTag);
      Logger.recordOutput("SingleTagAlign/filteredDistance", filteredDistance);
    }
    if (horizontalDistance != 0) {
      filteredHorizontal = yFilter.calculate(horizontalDistance);
      Logger.recordOutput("SingleTagAlign/filteredHorizontal", filteredHorizontal);
    }
    if (omega != 0) {
      filteredOmega = omegaFilter.calculate(omega);
      Logger.recordOutput("SingleTagAlign/filteredOmega", filteredOmega);
    }

    double omegaOutput = filteredOmega == 0 ? 0 : angleController.calculate(filteredOmega, 0);

    double distanceOutput = distToTag == 0 ? 0 : distanceController.calculate(filteredDistance, 2);

    double horizontalOutput = horizontalDistance == 0
        ? 0
        : horizontalController.calculate(filteredHorizontal, 0);

    // Convert to field relative speeds & send command
    double maxChassisSpeed = 4.5;
    ChassisSpeeds speeds = new ChassisSpeeds(
        distanceOutput * maxChassisSpeed,
        horizontalOutput * maxChassisSpeed,
        omegaOutput * maxChassisSpeed);

    // NOW DRIVE!!!
    s_Swerve.drive(speeds);

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
