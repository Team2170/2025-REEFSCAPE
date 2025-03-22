package frc.robot.Subsystems.AlgaeRemover;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.AlgaeRemover.Components.AlgaeRemoverIO;
import frc.robot.Subsystems.AlgaeRemover.Components.AlgaeRemoverIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/** The AlgaeRemover subsystem controls the climbing mechanism of the robot. */
public class AlgaeRemover extends SubsystemBase {
  private final AlgaeRemoverIO io;
  private final AlgaeRemoverIOInputsAutoLogged inputs = new AlgaeRemoverIOInputsAutoLogged();
  private final String SubystemName;

  /**
   * Constructs a AlgaeRemover subsystem.
   *
   * @param name The name of the subsystem for logging purposes.
   * @param io The AlgaeRemoverIO instance handling hardware interactions.
   */
  public AlgaeRemover(String name, AlgaeRemoverIO io) {
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
   * @param percent The percentage output to set the AlgaeRemover motor (-1.0 to 1.0).
   */
  public void setPercentOut(double percent) {
    io.setPercentOut(percent);
  }

  /** Stops the AlgaeRemover, setting the output to zero and maintaining position. */
  public void stop() {
    io.stop();
  }
}
