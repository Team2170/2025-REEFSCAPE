package frc.robot.Subsystems.Elevator.Components;

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.Utility.ElevatorState;

public class ElevatorIOSim implements ElevatorIO {

  private DCMotor gearBox = DCMotor.getFalcon500Foc(1);
  private CANcoder encoder;
  private CANcoderSimState encoderSim;

  private TalonFX motor;
  private TalonFX mFollower;
  private TalonFXSimState mMotorSim;
  private ElevatorSim elevatorSim;
  private ElevatorState desiredState = ElevatorState.UNKNOWN;
  private MotionMagicTorqueCurrentFOC positionRequest = new MotionMagicTorqueCurrentFOC(0);

  public ElevatorIOSim(int motorID, int motorFollowerId, String canbus, int encoderID) {
    encoder = new CANcoder(encoderID);
    encoderSim = encoder.getSimState();

    elevatorSim = new ElevatorSim(
        LinearSystemId.createElevatorSystem(gearBox, encoderID, encoderID, encoderID),
        gearBox,
        encoderID,
        encoderID,
        true,
        encoderID,
        null);

    motor = new TalonFX(motorID, canbus);
    mFollower = new TalonFX(motorFollowerId, canbus);

    mMotorSim = motor.getSimState();

  }

  public void updateInputs(ElevatorIOInputs inputs) {
    // Simulation Components
    mMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    elevatorSim.setInput(mMotorSim.getMotorVoltage());
    elevatorSim.update(0.2);
    // Get Elevator Details

    inputs.torqueCurrentAmps = motor.getTorqueCurrent().getValueAsDouble();
    inputs.velocityRotPerSec = motor.getVelocity().getValueAsDouble();
    inputs.rotPosition = Rotation2d.fromRotations(encoder.getPosition().getValueAsDouble());
    inputs.positionPercent = encoder.getPosition().getValueAsDouble() / Elevator.MAX_ROTATIONS.getRotations();
    inputs.aligned = Math.abs(encoder.getPosition().getValueAsDouble()
        - desiredState.pos.getRotations()) < Elevator.ELEVATOR_TOLERANCE.getRotations();
    inputs.motorConnected = motor.isConnected();
    inputs.encoderConnected = encoder.isConnected();
    inputs.controlMode = motor.getControlMode().getValue();
    inputs.positionRotations = inputs.rotPosition.getRotations();
    inputs.state = desiredState;
    inputs.heightMeters = inputs.positionRotations * Elevator.METERS_PER_ROTATION;
  }

  public void setDesiredState(ElevatorState state) {
    this.desiredState = state;
    motor.setControl(positionRequest.withPosition(state.pos.getRotations()));
  }

  public void setPercentOutput(double percent){
    motor.set(percent);
  }

  public void stop(){

  }
  public void hold(double hold){
    
  }
}