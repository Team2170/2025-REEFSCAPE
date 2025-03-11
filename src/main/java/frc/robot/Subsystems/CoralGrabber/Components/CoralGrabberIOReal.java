package frc.robot.Subsystems.CoralGrabber.Components;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
<<<<<<< HEAD
=======

>>>>>>> 9ffdd3ca3a5c258b4be2b0ffa522dfdbd3388ecd
import frc.robot.Subsystems.CoralGrabber.Utility.CoralGrabberConfiguration;

public class CoralGrabberIOReal implements CoralGrabberIO {
  private TalonFX mMotor;
  private CANrange frontSensor;
  private CANrange backSensor;
  private final DutyCycleOut request;
  private CoralGrabberConfiguration cfg;

<<<<<<< HEAD
  public CoralGrabberIOReal(int motorId, String motorBus, String sensorBus) {
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
    internalConfig.CurrentLimits.withStatorCurrentLimit(20);
    internalConfig.CurrentLimits.withStatorCurrentLimitEnable(true);
    mMotor.getConfigurator().apply(internalConfig);
  }

  public void updateInputs(CoralGrabberIOInputs inputs) {
    inputs.intakeVelocity = getVelocity();
    // inputs.frontSensorRange = frontSensor.getDistance().getValueAsDouble();
    // inputs.backSensorRange = backSensor.getDistance().getValueAsDouble();
  }
=======
    public CoralGrabberIOReal(int motorId, String motorBus, String sensorBus) {
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
        internalConfig.CurrentLimits.withStatorCurrentLimit(20);
        internalConfig.CurrentLimits.withStatorCurrentLimitEnable(
                true);
        mMotor.getConfigurator().apply(internalConfig);
    }

    public void updateInputs(CoralGrabberIOInputs inputs) {
        inputs.intakeVelocity = getVelocity();
        // inputs.frontSensorRange = frontSensor.getDistance().getValueAsDouble();
        // inputs.backSensorRange = backSensor.getDistance().getValueAsDouble();
    }
>>>>>>> 9ffdd3ca3a5c258b4be2b0ffa522dfdbd3388ecd

  public double getVelocity() {
    return mMotor.getVelocity().getValueAsDouble();
  }

  public void setIntakeSpeed(double speed) {
    mMotor.setControl(request.withOutput(speed));
  }

<<<<<<< HEAD
  public void stopIntake() {
    mMotor.stopMotor();
  }
=======

    public void stopIntake(){
        mMotor.stopMotor();
    }
>>>>>>> 9ffdd3ca3a5c258b4be2b0ffa522dfdbd3388ecd
}
