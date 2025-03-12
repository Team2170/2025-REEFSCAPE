package frc.robot.Subsystems.Elevator.Components;

import com.ctre.phoenix6.signals.ControlModeValue;
import frc.robot.Subsystems.Elevator.Utility.ElevatorState;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public ElevatorState state = ElevatorState.UNKNOWN;
    // LEFT INPUTS
    public double leftTorqueCurrentAmps = -1;
    public double leftPositionRotations = 0;
    public double leftVelocityRotPerSec = -1;
    public boolean leftMotorConnected = false;
    public boolean leftEncoderConnected = false;
    public ControlModeValue leftControlMode = ControlModeValue.DisabledOutput;
    public double leftPositionError = -1;
    // LEFT INPUTS
    public double rightTorqueCurrentAmps = -1;
    public double rightPositionRotations = 0;
    public double rightVelocityRotPerSec = -1;
    public boolean rightMotorConnected = false;
    public boolean rightEncoderConnected = false;
    public ControlModeValue rightControlMode = ControlModeValue.DisabledOutput;
    public double rightPositionError = -1;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setDesiredState(ElevatorState state) {}

  public default void setPercentOutput(double percent) {}

  public default void stop() {}

  public default void hold(double hold) {}

  public default double averagedPosition() {
    return 0.0;
  }
}
