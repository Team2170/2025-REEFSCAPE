package frc.robot.subsystems.Gripper.module;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.CTREConfigs;
import frc.robot.Constants;
import frc.robot.Constants.GripperConstants;
import frc.robot.subsystems.Gripper.GripperState;

public class GripperModule {
    private TalonFX mGripperMotorLeft;
    private TalonFX mGripperMotorRight;
    private CANcoder mGripperEncoderLeft;
    private CANcoder mGripperEncoderRight;

    public GripperModule() {
        mGripperMotorLeft = new TalonFX(GripperConstants.leftMotorId, Constants.canivorename);
        mGripperEncoderLeft = new CANcoder(GripperConstants.leftEncoderId, Constants.canivorename);

        mGripperMotorRight = new TalonFX(GripperConstants.rightMotorId, Constants.canivorename);
        mGripperEncoderRight = new CANcoder(GripperConstants.rightEncoderId, Constants.canivorename);

        // Gripper configuration
        CTREConfigs.basicConfiguration(mGripperEncoderLeft, mGripperMotorLeft, GripperConstants.leftEncoderId,
                GripperConstants.leftMotorId);

        CTREConfigs.basicConfiguration(mGripperEncoderRight, mGripperMotorRight, GripperConstants.rightEncoderId,
                GripperConstants.rightMotorId);

        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = GripperConstants.kP;
        slot0Configs.kI = GripperConstants.kI;
        slot0Configs.kD = GripperConstants.kD;
        slot0Configs.kS = GripperConstants.kS;
        slot0Configs.kV = GripperConstants.kV;

        mGripperMotorLeft.getConfigurator().apply(slot0Configs);
        mGripperMotorRight.setControl(new Follower(GripperConstants.leftMotorId, true));
    }

    public void setDesiredState(GripperState state) {
        // create a velocity closed-loop request, voltage output, slot 0 configs
        final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

        mGripperMotorLeft.setControl(m_request.withVelocity(state.speed));
        // TODO check sensor fault && some pid stuff needs to be done too
        // if(mGripperMotor.getFaultSensor)
    }

    public GripperState getState() {
        // TODO figure out proper states for each subsystem, this is just a generic
        // implementation
        return new GripperState(mGripperMotorLeft.getVelocity().getValueAsDouble());
    }

    public void stop() {
        mGripperMotorLeft.stopMotor();
        mGripperMotorRight.stopMotor();
    }
}
