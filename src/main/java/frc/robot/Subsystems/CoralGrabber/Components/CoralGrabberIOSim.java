package frc.robot.Subsystems.CoralGrabber.Components;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import BobcatLib.Hardware.Motors.SensorHelpers.InvertedWrapper;
import BobcatLib.Hardware.Motors.Utility.CTRE.PidControllerWrapper;
import frc.robot.Subsystems.CoralGrabber.Utility.CoralGrabberConfiguration;

public class CoralGrabberIOSim implements CoralGrabberIO {
    private TalonFX mMotor;
    private CANrange frontSensor;
    private CANrange backSensor;
    private final DutyCycleOut request;
    private CoralGrabberConfiguration cfg;

    public CoralGrabberIOSim(CoralGrabberConfiguration cfg) {
        this.cfg = cfg;
        mMotor = new TalonFX(cfg.motorJson.motorId, cfg.motorJson.motorBus);
        configMotor();
        frontSensor = new CANrange(cfg.frontIntakeJson.sensorId, cfg.frontIntakeJson.sensorBus);
        backSensor = new CANrange(cfg.backIntakeJson.sensorId, cfg.backIntakeJson.sensorBus);
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
        internalConfig.Feedback.withSensorToMechanismRatio(cfg.motorJson.motorToGearRatio);
        internalConfig.Slot0 = new PidControllerWrapper(new Slot0Configs())
                .with_kP(cfg.motorJson.kP)
                .with_kI(cfg.motorJson.kI)
                .with_kD(cfg.motorJson.kD)
                .getSlot0Config();
        internalConfig.CurrentLimits.withStatorCurrentLimit(cfg.motorJson.statorCurrentLimit);
        internalConfig.CurrentLimits.withStatorCurrentLimitEnable(
                true);
        mMotor.getConfigurator().apply(internalConfig);
    }

    public void updateInputs(CoralGrabberIOInputs inputs) {
        inputs.intakeVelocity = getVelocity();
        inputs.frontSensorRange = frontSensor.getDistance().getValueAsDouble();
        inputs.backSensorRange = backSensor.getDistance().getValueAsDouble();
        inputs.isIntaked = hasIntaked(inputs.frontSensorRange, inputs.backSensorRange);
    }

    public double getVelocity() {
        return mMotor.getVelocity().getValueAsDouble();
    }

    public void setIntakeSpeed(double speed) {
        mMotor.setControl(request.withOutput(speed));
    }

    public boolean hasIntaked(double frontRange, double backRange) {
        boolean isFrontEngaged = isTripped(frontRange, cfg.frontIntakeJson.intakedRange);
        boolean isBackEngaged = isTripped(backRange, cfg.backIntakeJson.intakedRange);
        if (isFrontEngaged && isBackEngaged) {
            return true;
        }
        return false;
    }

    public boolean isTripped(double range, double threshold) {
        if (range < threshold) {
            return true;
        }
        return false;
    }

    public void stopIntake(){
        mMotor.stopMotor();
    }
}
