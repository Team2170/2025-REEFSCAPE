package frc.robot.Subsystems.CoralGrabber.Components;

import org.littletonrobotics.junction.AutoLog;

public interface CoralGrabberIO {
  @AutoLog
  public static class CoralGrabberIOInputs {
    public double frontSensorRange = 0;
    public double backSensorRange = 0;
    public double intakeVelocity = 0;
    public boolean isIntaked = false;
  }

  public default void updateInputs(CoralGrabberIOInputs inputs) {}

  public default boolean hasIntaked() {
    return false;
  }

  public default void setIntakeSpeed(double speed) {}

  public default void stopMotor() {}
}
