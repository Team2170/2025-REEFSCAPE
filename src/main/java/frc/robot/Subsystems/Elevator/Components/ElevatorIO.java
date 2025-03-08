package frc.robot.Subsystems.Elevator.Components;

import com.ctre.phoenix6.signals.ControlModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Subsystems.Elevator.Utility.ElevatorState;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public ElevatorState state = ElevatorState.UNKNOWN;
    public Rotation2d rotPosition = Rotation2d.kZero;
    public double positionRotations = 0;
    public double velocityRotPerSec = -1;
    public double torqueCurrentAmps = -1;
    public double positionPercent = -1;
    public double heightMeters = -1;
    /** is the elevator at its desired state? */
    public boolean aligned = false;

    public ControlModeValue controlMode = ControlModeValue.DisabledOutput;

    public boolean motorConnected = false;
    public boolean encoderConnected = false;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {
  }

  public default void setDesiredState(ElevatorState state) {
  }

  public default void setPercentOutput(double percent){

  }
  public default void stop(){

  }
  public default void hold(double hold){
    
  }
  public default double averagedPosition(){
    return 0.0;
  }
}