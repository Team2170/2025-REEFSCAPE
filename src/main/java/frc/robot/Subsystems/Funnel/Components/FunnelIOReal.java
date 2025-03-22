package frc.robot.Subsystems.Funnel.Components;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.Constants;

/** Implementation of FunnelIO for real hardware, using a TalonFX motor controller. */
public class FunnelIOReal implements FunnelIO {
  private TalonFX mMotor;
  public static final double GEAR_RATIO = 12.00; // SET THIS UP
  private DutyCycleOut request;
  private PositionDutyCycle holdPosRequest;

  /**
   * Constructs a FunnelIOReal instance with the given configuration.
   *
   * @param cfg The FunnelConfiguration object containing configuration parameters.
   */
  public FunnelIOReal() {
    mMotor = new TalonFX(Constants.FunnelConstants.funnelMotorId);
    configMotor();
    request = new DutyCycleOut(0).withEnableFOC(true);
  }

  /** Configures the motor with the provided parameters. */
  public void configMotor() {
    TalonFXConfiguration internalConfig = new TalonFXConfiguration();
    internalConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    internalConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    internalConfig.Feedback.withSensorToMechanismRatio(GEAR_RATIO);
    internalConfig.CurrentLimits.withStatorCurrentLimit(120);
    internalConfig.CurrentLimits.withStatorCurrentLimitEnable(true);
    // Apply all settings.
    mMotor.getConfigurator().apply(internalConfig);
  }

  /**
   * Updates the input state with the current sensor values.
   *
   * @param inputs The FunnelIOInputs object to update.
   */
  public void updateInputs(FunnelIOInputs inputs) {
    inputs.TorqueCurrentAmps = mMotor.getTorqueCurrent().getValueAsDouble();
    inputs.VelocityRotPerSec = mMotor.getVelocity().getValueAsDouble();
    inputs.MotorConnected = mMotor.isConnected();
    inputs.ControlMode = mMotor.getControlMode().getValue();
    inputs.PositionError = mMotor.getClosedLoopError().getValueAsDouble();
  }

  /**
   * Sets the motor output as a percentage of total power.
   *
   * @param percent The percentage output to set the Funnel motor (-1.0 to 1.0).
   */
  public void setPercentOut(double percent) {
    mMotor.setControl(request.withOutput(percent));
  }

  /**
   * Holds the Funnel at a specific position.
   *
   * @param rot The target position in rotations.
   */
  public void hold(double rot) {
    mMotor.setControl(holdPosRequest.withPosition(rot));
  }

  /** Stops the Funnel motor. */
  public void stop() {
    mMotor.stopMotor();
  }
}
