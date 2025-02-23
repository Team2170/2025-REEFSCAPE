package frc.robot.Subsystems.Climber.Components;

import org.littletonrobotics.junction.AutoLog;

/**
 * Interface for the Climber subsystem's input/output operations.
 */
public interface ClimberIO {
    
    /**
     * Class representing input data for the Climber subsystem.
     */
    @AutoLog
    public static class ClimberIOInputs {
        
        /**
         * The current position of the climber in rotations.
         */
        public double climberPosition = 0;
        
        /**
         * The current velocity of the climber motor.
         */
        public double climberVelocity = 0;
        
        /**
         * Whether the climber is currently hanging.
         */
        public boolean isHanging = false;
    }

    /**
     * Updates the sensor inputs for the climber.
     *
     * @param inputs The ClimberIOInputs object to be updated.
     */
    public default void updateInputs(ClimberIOInputs inputs) {}

    /**
     * Sets the climber motor output as a percentage of total power.
     *
     * @param percent The percentage output to set the climber motor (-1.0 to 1.0).
     */
    public default void setPercentOut(double percent) {}

    /**
     * Holds the climber at a specified position.
     *
     * @param rot The target position in rotations.
     */
    public default void holdPos(double rot) {}

    /**
     * Stops the climber motor.
     */
    public default void stop() {}
}