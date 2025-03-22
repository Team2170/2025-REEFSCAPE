package frc.robot.Subsystems.Funnel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Funnel.Components.FunnelIO;
import frc.robot.Subsystems.Funnel.Components.FunnelIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/** The Funnel subsystem controls the climbing mechanism of the robot. */
public class Funnel extends SubsystemBase {
  private final FunnelIO io;
  private final FunnelIOInputsAutoLogged inputs = new FunnelIOInputsAutoLogged();
  private final String SubystemName;

  /**
   * Constructs a Funnel subsystem.
   *
   * @param name The name of the subsystem for logging purposes.
   * @param io The FunnelIO instance handling hardware interactions.
   */
  public Funnel(String name, FunnelIO io) {
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
   * @param percent The percentage output to set the Funnel motor (-1.0 to 1.0).
   */
  public void setPercentOut(double percent) {
    io.setPercentOut(percent);
  }

  /** Stops the Funnel, setting the output to zero and maintaining position. */
  public void stop() {
    io.stop();
  }
}
