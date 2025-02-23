package frc.robot.Subsystems.Climber.Utility;


/**
 * Represents the motor configuration for the Climber subsystem.
 */
public class ClimberMotorJson {
    
    /**
     * Indicates whether the motor is inverted.
     */
    public boolean isInverted;
    
    /**
     * The ratio of motor rotations to gear rotations.
     */
    public double motorToGearRatio;
    
    /**
     * Proportional gain for motor PID control.
     */
    public double kP;
    
    /**
     * Integral gain for motor PID control.
     */
    public double kI;
    
    /**
     * Derivative gain for motor PID control.
     */
    public double kD;
    
    /**
     * The stator current limit for the motor.
     */
    public double statorCurrentLimit;
    
    /**
     * The unique identifier for the motor.
     */
    public int motorId;
    
    /**
     * The CAN bus the motor controller is connected to.
     */
    public String motorBus;
    
    /**
     * The lower range limit for the motor rotations.
     */
    public double lowerRangeRotations;
    
    /**
     * The upper range limit for the motor rotations.
     */
    public double upperRangeRotations;
}
