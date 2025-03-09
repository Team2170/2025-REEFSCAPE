package frc.robot.Subsystems.CoralGrabber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.CoralGrabber.Components.CoralGrabberIO;
import frc.robot.Subsystems.CoralGrabber.Components.CoralGrabberIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class CoralGrabber extends SubsystemBase {
  private final CoralGrabberIO io;
  private final CoralGrabberIOInputsAutoLogged inputs = new CoralGrabberIOInputsAutoLogged();
  private final String SubystemName;
  // Set Some Speed to operate the Grabber Intake Motors
  public CoralGrabber(String name, CoralGrabberIO io) {
    this.SubystemName = name;
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(SubystemName, inputs);
  }

  public void setIntakeSpeed(double speed) {
    io.setIntakeSpeed(speed);
  }

  public void stopIntake() {
    io.stopMotor();
  }
}
