package frc.robot.Subsystems.Climber.Components;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.Constants;

/** Implementation of ClimberIO for real hardware, using a TalonFX motor controller. */
public class ClimberIOReal implements ClimberIO {
  private TalonFX mMotor;
  public static final double GEAR_RATIO = 12.00; // SET THIS UP
  private DutyCycleOut request;
  private PositionDutyCycle holdPosRequest;

  /**
   * Constructs a ClimberIOReal instance with the given configuration.
   *
   * @param cfg The ClimberConfiguration object containing configuration parameters.
   */
  public ClimberIOReal() {
    mMotor = new TalonFX(Constants.ClimberConstants.climberMotorId);
    configMotor();
    request = new DutyCycleOut(0).withEnableFOC(true);
  }

  /** Configures the motor with the provided parameters. */
  public void configMotor() {
    TalonFXConfiguration internalConfig = new TalonFXConfiguration();
    internalConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    internalConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    internalConfig.Feedback.withSensorToMechanismRatio(GEAR_RATIO);
    internalConfig.CurrentLimits.withStatorCurrentLimit(120);
    internalConfig.CurrentLimits.withStatorCurrentLimitEnable(true);
    // Apply all settings.
    mMotor.getConfigurator().apply(internalConfig);
  }

  /**
   * Updates the input state with the current sensor values.
   *
   * @param inputs The ClimberIOInputs object to update.
   */
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.TorqueCurrentAmps = mMotor.getTorqueCurrent().getValueAsDouble();
    inputs.VelocityRotPerSec = mMotor.getVelocity().getValueAsDouble();
    inputs.MotorConnected = mMotor.isConnected();
    inputs.ControlMode = mMotor.getControlMode().getValue();
    inputs.PositionError = mMotor.getClosedLoopError().getValueAsDouble();
  }

  /**
   * Sets the motor output as a percentage of total power.
   *
   * @param percent The percentage output to set the climber motor (-1.0 to 1.0).
   */
  public void setPercentOut(double percent) {
    mMotor.setControl(request.withOutput(percent));
  }

  /**
   * Holds the climber at a specific position.
   *
   * @param rot The target position in rotations.
   */
  public void hold(double rot) {
    mMotor.setControl(holdPosRequest.withPosition(rot));
  }

  /** Stops the climber motor. */
  public void stop() {
    mMotor.stopMotor();
  }
}
