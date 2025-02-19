package frc.robot.subsystems.Climber.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.CTREConfigs;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber.ClimberState;

public class ClimberModule {
    private TalonFX mClimberMotor;
    private CANcoder mClimberEncoder;

    public ClimberModule() {
        mClimberMotor = new TalonFX(ClimberConstants.climberId, Constants.canivorename);
        mClimberEncoder = new CANcoder(ClimberConstants.encoderId, Constants.canivorename);

        // Configuration
       CTREConfigs.basicConfiguration(mClimberEncoder, mClimberMotor, ClimberConstants.encoderId, ClimberConstants.climberId);
        mClimberMotor.getConfigurator().setPosition(0.0);
    }

    /**
     * @param desiredState
     * @param isOpenLoop
     *                     Sets the desired state of the module.
     */
    public void setDesiredState(ClimberState state) {
        mClimberMotor.setPosition(state.rotations.getRotations());
    }

    public Rotation2d getRotations() {
        // signals can be implicitly refreshed
        // TODO find out if there is a better way to find the position from the encoder
        // reading
        double rotations = mClimberMotor.getPosition().getValue().in(Units.Rotations);
        return Rotation2d.fromRotations(rotations);
    }

    /**
     * @return Intake given module velocity and angle.
     */
    public ClimberState getState() {
        return new ClimberState(getRotations());
    }

    /**
     * Stop the motor.
     */
    public void stop() {
        mClimberMotor.stopMotor();
    }
}