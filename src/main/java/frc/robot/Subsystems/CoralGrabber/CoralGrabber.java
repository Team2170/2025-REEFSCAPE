package frc.robot.Subsystems.CoralGrabber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.CoralGrabber.Components.CoralGrabberIO;
import frc.robot.Subsystems.CoralGrabber.Components.CoralGrabberIOInputsAutoLogged;


public class CoralGrabber extends SubsystemBase {
    private final CoralGrabberIO io;
    private final CoralGrabberIOInputsAutoLogged inputs = new CoralGrabberIOInputsAutoLogged();
    private final String SubystemName;
    // Set Some Speed to operate the Grabber Intake Motors
    public CoralGrabber(String name , CoralGrabberIO io){
        this.SubystemName = name;
        this.io = io;
    }
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(SubystemName, inputs);
    }
    public void setIntakeSpeed(double speed){
        io.setIntakeSpeed(speed);
    }
    public boolean hasIntaked(){
        return inputs.isIntaked;
    }
    public void stopIntake(){
        io.stopMotor();
    }
}
