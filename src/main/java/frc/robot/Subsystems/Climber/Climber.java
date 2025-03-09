package frc.robot.Subsystems.Climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Climber.Components.ClimberIO;
import frc.robot.Subsystems.Climber.Components.ClimberIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/** The Climber subsystem controls the climbing mechanism of the robot. */
public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private final String SubystemName;

  /**
   * Constructs a Climber subsystem.
   *
   * @param name The name of the subsystem for logging purposes.
   * @param io The ClimberIO instance handling hardware interactions.
   */
  public Climber(String name, ClimberIO io) {
    this.SubystemName = name;
    this.io = io;
  }

  /**
   * Periodic method called once per scheduler run. Updates sensor inputs and maintains position
   * when necessary.
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(SubystemName, inputs);
  }

  /**
   * Sets the motor output as a percentage of total power.
   *
   * @param percent The percentage output to set the climber motor (-1.0 to 1.0).
   */
  public void setPercentOut(double percent) {
    io.setPercentOut(percent);
  }

  /** Stops the climber, setting the output to zero and maintaining position. */
  public void stop() {
    io.stop();
  }
}
