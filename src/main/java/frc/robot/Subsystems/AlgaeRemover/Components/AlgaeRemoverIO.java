package frc.robot.Subsystems.AlgaeRemover.Components;

import com.ctre.phoenix6.signals.ControlModeValue;
import org.littletonrobotics.junction.AutoLog;

/** Interface for the AlgaeRemover subsystem's input/output operations. */
public interface AlgaeRemoverIO {

  /** Class representing input data for the AlgaeRemover subsystem. */
  @AutoLog
  public static class AlgaeRemoverIOInputs {
    // LEFT INPUTS
    public double TorqueCurrentAmps = -1;
    public double VelocityRotPerSec = -1;
    public boolean MotorConnected = false;
    public ControlModeValue ControlMode = ControlModeValue.DisabledOutput;
    public double PositionError = -1;
  }

  /**
   * Updates the sensor inputs for the AlgaeRemover.
   *
   * @param inputs The AlgaeRemoverIOInputs object to be updated.
   */
  public default void updateInputs(AlgaeRemoverIOInputs inputs) {}

  /**
   * Sets the AlgaeRemover motor output as a percentage of total power.
   *
   * @param percent The percentage output to set the AlgaeRemover motor (-1.0 to 1.0).
   */
  public default void setPercentOut(double percent) {}

  /**
   * Holds the AlgaeRemover at a specified position.
   *
   * @param rot The target position in rotations.
   */
  public default void hold(double rot) {}

  /** Stops the AlgaeRemover motor. */
  public default void stop() {}
}
