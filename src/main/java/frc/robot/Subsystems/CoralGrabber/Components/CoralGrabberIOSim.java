package frc.robot.Subsystems.CoralGrabber.Components;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Subsystems.CoralGrabber.Utility.CoralGrabberConfiguration;

public class CoralGrabberIOSim implements CoralGrabberIO {
  private TalonFX mMotor;
  private CANrange frontSensor;
  private CANrange backSensor;
  private final DutyCycleOut request;
  private CoralGrabberConfiguration cfg;

  public CoralGrabberIOSim(int motorId, String motorBus, String sensorBus) {
    this.cfg = new CoralGrabberConfiguration();
    mMotor = new TalonFX(motorId, motorBus);
    configMotor();
    // frontSensor = new CANrange(cfg.frontIntakeJson.sensorId, sensorBus);
    // backSensor = new CANrange(cfg.backIntakeJson.sensorId, sensorBus);
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
    // internalConfig.Slot0 =
    //     new PidControllerWrapper(new Slot0Configs())
    //         .with_kP(cfg.motorJson.kP)
    //         .with_kI(cfg.motorJson.kI)
    //         .with_kD(cfg.motorJson.kD)
    //         .getSlot0Config();
    internalConfig.CurrentLimits.withStatorCurrentLimit(20);
    internalConfig.CurrentLimits.withStatorCurrentLimitEnable(true);
    mMotor.getConfigurator().apply(internalConfig);
  }

  public void updateInputs(CoralGrabberIOInputs inputs) {
    inputs.intakeVelocity = getVelocity();
    // inputs.frontSensorRange = frontSensor.getDistance().getValueAsDouble();
    // inputs.backSensorRange = backSensor.getDistance().getValueAsDouble();
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
