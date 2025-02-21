// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.ArrayList;
import java.util.List;

import org.dyn4j.world.Island;
import org.ejml.equation.IntegerSequence.Range;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import BobcatLib.Hardware.Sensors.SpatialSensor.Spatial;
import BobcatLib.Hardware.Sensors.SpatialSensor.SpatialTOF;
import BobcatLib.Hardware.Sensors.SpatialSensor.Components.CANRange;
import BobcatLib.Hardware.Sensors.SpatialSensor.Components.RangeSensor;
import BobcatLib.Hardware.Sensors.SpatialSensor.Components.SimTOF;
import BobcatLib.Hardware.Sensors.SpatialSensor.Utility.DistanceMode;
import BobcatLib.Hardware.Sensors.SpatialSensor.Utility.DistanceMode.modes;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.SwerveDrive;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignRobot extends Command {
  Spatial distanceSensing;
  SwerveDrive s_Swerve;
  private boolean isligned = false;
  private SpatialTOF stof;

  /** Creates a new AlignRobot. */
  public AlignRobot(SwerveDrive swerve) {
    this.s_Swerve = swerve;
    List<RangeSensor> distanceSensors = new ArrayList<RangeSensor>();
    distanceSensors.add(new CANRange(5, new DistanceMode(modes.LONG), 20, "CANt_open_file"));
    distanceSensors.add(new CANRange(6, new DistanceMode(modes.LONG), 20, "CANt_open_file"));
    stof = new SpatialTOF(distanceSensors);

    distanceSensing = new Spatial(stof);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Logger.recordOutput("Spatial",true);
    double leftRange = distanceSensing.getDistances().get("left");
    double rightRange = distanceSensing.getDistances().get("right");

    if (leftRange > rightRange) {
      // Turn CCW
      boolean isOpenLoop = true;
      Translation2d linearTranslation = new Translation2d(0, 0);
      double rotation = s_Swerve.getHeading().getDegrees() + 2; // UPDATE WITH ROTATION YOU ARE TURNING
      s_Swerve.drive(linearTranslation, rotation, isOpenLoop,
          s_Swerve.getHeading(),
          s_Swerve.getPose());
    } else if (rightRange > leftRange) {
      // Turn CW
      boolean isOpenLoop = true;
      Translation2d linearTranslation = new Translation2d(0, 0);
      double rotation = s_Swerve.getHeading().getDegrees() - 2; // UPDATE WITH ROTATION YOU ARE TURNING
      s_Swerve.drive(linearTranslation, rotation, isOpenLoop,
          s_Swerve.getHeading(),
          s_Swerve.getPose());
    } else {
      isligned = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return isligned;
  }
}
