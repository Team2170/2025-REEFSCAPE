package frc.robot.Subsystems.Funnel.Components;

import com.ctre.phoenix6.signals.ControlModeValue;
import org.littletonrobotics.junction.AutoLog;

/** Interface for the Funnel subsystem's input/output operations. */
public interface FunnelIO {

  /** Class representing input data for the Funnel subsystem. */
  @AutoLog
  public static class FunnelIOInputs {
    // LEFT INPUTS
    public double TorqueCurrentAmps = -1;
    public double VelocityRotPerSec = -1;
    public boolean MotorConnected = false;
    public ControlModeValue ControlMode = ControlModeValue.DisabledOutput;
    public double PositionError = -1;
  }

  /**
   * Updates the sensor inputs for the Funnel.
   *
   * @param inputs The FunnelIOInputs object to be updated.
   */
  public default void updateInputs(FunnelIOInputs inputs) {}

  /**
   * Sets the Funnel motor output as a percentage of total power.
   *
   * @param percent The percentage output to set the Funnel motor (-1.0 to 1.0).
   */
  public default void setPercentOut(double percent) {}

  /**
   * Holds the Funnel at a specified position.
   *
   * @param rot The target position in rotations.
   */
  public default void hold(double rot) {}

  /** Stops the Funnel motor. */
  public default void stop() {}
}
