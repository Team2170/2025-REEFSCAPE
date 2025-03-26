package frc.robot.Subsystems.CoralGrabber.Components;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class CoralGrabberIOSim implements CoralGrabberIO {
  private TalonFX mMotor;
  private final DutyCycleOut request;

  public CoralGrabberIOSim(int motorId, String motorBus) {
    mMotor = new TalonFX(motorId, motorBus);
    configMotor();
    request = new DutyCycleOut(0).withEnableFOC(true);
  }

  /**
   * Configures the motor with the provided parameters.
   *
   * @param cfg The MotorConfigs object containing configuration parameters.
   */
  public void configMotor() {
    TalonFXConfiguration internalConfig = new TalonFXConfiguration();
    internalConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    internalConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    internalConfig.Feedback.withSensorToMechanismRatio(1);
    internalConfig.CurrentLimits.withStatorCurrentLimit(1);
    internalConfig.CurrentLimits.withStatorCurrentLimitEnable(true);
    mMotor.getConfigurator().apply(internalConfig);
  }

  public void updateInputs(CoralGrabberIOInputs inputs) {
    inputs.intakeVelocity = getVelocity();
  }

  public double getVelocity() {
    return mMotor.getVelocity().getValueAsDouble();
  }

  public void setIntakeSpeed(double speed) {
    mMotor.setControl(request.withOutput(speed));
  }

  public void stopIntake() {
    mMotor.stopMotor();
  }
}
