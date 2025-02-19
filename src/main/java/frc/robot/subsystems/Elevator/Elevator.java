package frc.robot.subsystems.Elevator;

import org.dyn4j.geometry.Rotation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator.module.ElevatorModule;

public class Elevator extends SubsystemBase {
    private ElevatorModule module;

    public Elevator() {
        module = new ElevatorModule();
    }

     @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator " + ElevatorConstants.leftMotorId + " Encoder (rot)",
                module.getState().rotations.getRotations());
    }

    public ElevatorModule getModule() {
        return module;
    }


    public void setElevatorPosition(double position) {
        module.setDesiredState(new ElevatorState(0, Rotation2d.fromRotations(position)));
    }

    public void stopElevator() {
        moduleLeft.stop();
        moduleRight.stop();
    }


}
