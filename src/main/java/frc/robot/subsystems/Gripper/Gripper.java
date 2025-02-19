package frc.robot.subsystems.Gripper;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Gripper.module.GripperModule;

public class Gripper extends SubsystemBase {
    private GripperModule module;

    public Gripper() {
        module = new GripperModule();
    }

     @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator " + ElevatorConstants.leftMotorId + " Encoder (rot)",
                module.getState().speed);
    }

    public GripperModule getModule() {
        return module;
    }


    public void setGripperSpeed(double speed) {
        module.setDesiredState(new GripperState(speed));
    }

    public void stopElevator() {
        module.stop();
    }
}
