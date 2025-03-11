package frc.robot.Subsystems.Elevator;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Elevator.Components.ElevatorIO;
import frc.robot.Subsystems.Elevator.Components.ElevatorIOInputsAutoLogged;
import frc.robot.Subsystems.Elevator.Utility.ElevatorState;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  public static final Rotation2d ELEVATOR_TOLERANCE = Rotation2d.fromDegrees(3); // Tune as Needed
  /** the number of rotations the encoder spins when it is at the top */
  public static final Rotation2d MAX_ROTATIONS = Rotation2d.fromRotations(142); // SET ME UP!!!

  public static final Distance ELEVATOR_MAX_HEIGHT = Inches.of(0); // SET ME UP!!!

  /** the number of output shaft rotations per meter of elevator travel */
  public static final double ROTATIONS_PER_METER = 0.00; // SET ME UP!!!

  public static final double METERS_PER_ROTATION = 1 / ROTATIONS_PER_METER;

  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public static final double PERCENT_OUTPUT = 0.3;
  public static final double HOLD_OUTPUT = 0.005;

  private final Alert motorDisconnectedAlert =
      new Alert("Elevator motor disconnected!", Alert.AlertType.kWarning);
  private final Alert encoderDisconnectedAlert =
      new Alert("Elevator encoder disconnected!", Alert.AlertType.kWarning);

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    motorDisconnectedAlert.set(!inputs.leftMotorConnected || !inputs.rightMotorConnected);
    encoderDisconnectedAlert.set(!inputs.leftEncoderConnected || !inputs.rightEncoderConnected);
  }

  public void setState(ElevatorState desiredState) {
    io.setDesiredState(desiredState);
  }

<<<<<<< HEAD
  public void setPercentOutput(double percent) {
=======
  public void setPercentOutput(double percent){
>>>>>>> 9ffdd3ca3a5c258b4be2b0ffa522dfdbd3388ecd
    io.setPercentOutput(percent);
  }

  public ElevatorState getState() {
    return inputs.state;
  }

  public void stop() {
    io.stop();
  }

  public void hold(double hold) {
    io.hold(hold);
  }
<<<<<<< HEAD
}
=======

  /**
   * @param distance
   * @return a {@code Rotation2d} containing the number of rotations necessary to acheive {@code
   *     distance} units of travel, this is useful for adjusting setpoints in a more semantically
   *     meaningful way, i.e. if a setpoint is an inch off, you can add {@code
   *     distanceToElevatorRotations(Inches.of(1))} to it
   */
  public static Rotation2d distanceToElevatorRotations(Distance distance) {
    return Rotation2d.fromRotations(distance.in(Meters) * ROTATIONS_PER_METER);
  }

  public void stop(){
    io.stop();
  }
  public void hold(double hold){
    io.hold(hold);
  }
}
>>>>>>> 9ffdd3ca3a5c258b4be2b0ffa522dfdbd3388ecd
