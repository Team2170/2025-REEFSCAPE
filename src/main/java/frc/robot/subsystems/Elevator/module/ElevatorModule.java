package frc.robot.subsystems.Elevator.module;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import frc.robot.CTREConfigs;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator.ElevatorState;

public class ElevatorModule {
    public static final double GEAR_RATIO = 1.0 / 1; // TODO change to correct gear ratio
    private MotionMagicTorqueCurrentFOC positionRequest = new MotionMagicTorqueCurrentFOC(0);

    private ElevatorState currentState;
    private TalonFX mElevatorMotor;
    private CANcoder mElevatorEncoder;

    public ElevatorModule() {
        mElevatorMotor = new TalonFX(ElevatorConstants.leftMotorId, Constants.canivorename); // TODO change to be
                                                                                             // parameterized so we can
                                                                                             // apply both elft and
                                                                                             // right
        mElevatorEncoder = new CANcoder(ElevatorConstants.leftEncoderId, Constants.canivorename);

        // Elevator configuration
        CTREConfigs.basicConfiguration(mElevatorEncoder, mElevatorMotor, ElevatorConstants.leftEncoderId,
                ElevatorConstants.leftMotorId);

        var slot0Configs = new Slot0Configs();
        // no kV or kA because this is torque current
        slot0Configs.kP = ElevatorConstants.kP;
        slot0Configs.kI = ElevatorConstants.kI;
        slot0Configs.kD = ElevatorConstants.kD;
        slot0Configs.kS = ElevatorConstants.kS; // TODO does this need kS, since we are PIDing around position
        slot0Configs.kG = ElevatorConstants.kG;
        currentState = ElevatorState.CORAL_L4; // TODO change to initial state of robot
        mElevatorMotor.getConfigurator().apply(slot0Configs);
    }

    public void setDesiredState(ElevatorState state) {
        PositionVoltage request = new PositionVoltage(0).withSlot(0);
        mElevatorMotor.setControl(request.withPosition(state.pos.getRotations()));
        currentState = state;
        // TODO check sensor fault && some pid stuff needs to be done too
        // if(mElevatorMotor.getFaultSensor)
    }

    public ElevatorState getState() {
        return currentState;

    }

    public void stop() {
        mElevatorMotor.stopMotor();
    }
}
