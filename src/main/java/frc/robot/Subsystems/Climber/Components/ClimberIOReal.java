package frc.robot.Subsystems.Climber.Components;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import BobcatLib.Hardware.Motors.Utility.CTRE.PidControllerWrapper;
import frc.robot.Subsystems.Climber.Utility.ClimberConfiguration;

/**
 * Implementation of ClimberIO for real hardware, using a TalonFX motor controller.
 */
public class ClimberIOReal implements ClimberIO {
    private TalonFX mMotor;
    private CANrange frontSensor;
    private CANrange backSensor;
    private ClimberConfiguration cfg;

    private DutyCycleOut request;
    private PositionDutyCycle holdPosRequest;

    /**
     * Constructs a ClimberIOReal instance with the given configuration.
     *
     * @param cfg The ClimberConfiguration object containing configuration parameters.
     */
    public ClimberIOReal(ClimberConfiguration cfg) {
        this.cfg = cfg;
        mMotor = new TalonFX(cfg.motorJson.motorId, cfg.motorJson.motorBus);
        configMotor();
        request = new DutyCycleOut(0).withEnableFOC(true);
    }

    /**
     * Configures the motor with the provided parameters.
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
        internalConfig.CurrentLimits.withStatorCurrentLimitEnable(true);
        
        // Set software limits for motor rotation.
        internalConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        internalConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = cfg.motorJson.upperRangeRotations;
        internalConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        internalConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = cfg.motorJson.lowerRangeRotations;
        
        // Apply all settings.
        mMotor.getConfigurator().apply(internalConfig);
    }

    /**
     * Updates the input state with the current sensor values.
     *
     * @param inputs The ClimberIOInputs object to update.
     */
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberVelocity = getVelocity();
        inputs.climberPosition = getPosition();
        inputs.isHanging = hasClimbed();
    }

    /**
     * Determines if the climber has reached the target climb rotations.
     *
     * @return true if the climber has climbed to the target, false otherwise.
     */
    public boolean hasClimbed() {
        return getPosition() >= cfg.climbRotations;
    }

    /**
     * Gets the current velocity of the climber motor.
     *
     * @return The velocity in appropriate units.
     */
    public double getVelocity() {
        return mMotor.getVelocity().getValueAsDouble();
    }

    /**
     * Gets the current position of the climber motor.
     *
     * @return The position in rotations.
     */
    public double getPosition() {
        return mMotor.getPosition().getValueAsDouble();
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
    public void holdPos(double rot) {
        mMotor.setControl(holdPosRequest.withPosition(rot));
    }

    /**
     * Stops the climber motor.
     */
    public void stop() {
        mMotor.stopMotor();
    }
}
