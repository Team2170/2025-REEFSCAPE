package frc.robot.Subsystems.Elevator.Utility;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Subsystems.Elevator.Elevator;

public enum ElevatorState {
    UNKNOWN(Rotation2d.fromRotations(0)), // picking up coral with the end effector
    CORAL_L1(Rotation2d.fromRotations(0)), // elevator pos doesnt matter for score or prep
    CORAL_L2(Rotation2d.fromRotations(1)),
    CORAL_L3(Rotation2d.fromRotations(2)),
    CORAL_L4(Rotation2d.fromRotations(3));
  ElevatorState(Rotation2d pos) {
    this.pos = pos;
    heightMeters = pos.getRotations() * Elevator.METERS_PER_ROTATION;
    height = Meters.of(heightMeters);
  }

  public Rotation2d pos;
  public Distance height;
  public double heightMeters;
}
