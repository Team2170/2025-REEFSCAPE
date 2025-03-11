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
import frc.robot.Constants.Constants;
import frc.robot.Subsystems.Elevator.Utility.ElevatorState;

public class ElevatorIOSim implements ElevatorIO {

  private DCMotor gearBox = DCMotor.getFalcon500Foc(1);
  private CANcoder mLeftEncoder;
  private CANcoder mRightEncoder;
  private CANcoderSimState encoderSim;

  private TalonFX leftMotor;
  private TalonFX rightMotor;
  private TalonFXSimState mMotorSim;
  private ElevatorSim elevatorSim;
  private ElevatorState desiredState = ElevatorState.UNKNOWN;
  private MotionMagicTorqueCurrentFOC positionRequest = new MotionMagicTorqueCurrentFOC(0);

  public ElevatorIOSim(int motorID, int motorFollowerId, String canbus, int encoderID) {
    mLeftEncoder = new CANcoder(Constants.ElevatorConstants.elevatorMasterCancoderId);
    mRightEncoder = new CANcoder(Constants.ElevatorConstants.elevatorFollowerCancoderId);
    encoderSim = mLeftEncoder.getSimState();

    elevatorSim =
        new ElevatorSim(
            LinearSystemId.createElevatorSystem(gearBox, encoderID, encoderID, encoderID),
            gearBox,
            encoderID,
            encoderID,
            true,
            encoderID,
            null);

    leftMotor = new TalonFX(motorID, canbus);
    rightMotor = new TalonFX(motorFollowerId, canbus);

    mMotorSim = leftMotor.getSimState();
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    // Simulation Components
    mMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    elevatorSim.setInput(mMotorSim.getMotorVoltage());
    elevatorSim.update(0.2);
    // Get Elevator Details
    inputs.state = desiredState;
    // Left Logging Values
    inputs.leftTorqueCurrentAmps = leftMotor.getTorqueCurrent().getValueAsDouble();
    inputs.leftPositionRotations =
        Rotation2d.fromRotations(mLeftEncoder.getPosition().getValueAsDouble()).getRotations();
    inputs.leftVelocityRotPerSec = leftMotor.getVelocity().getValueAsDouble();
    inputs.leftMotorConnected = leftMotor.isConnected();
    inputs.leftEncoderConnected = mLeftEncoder.isConnected();
    inputs.leftControlMode = leftMotor.getControlMode().getValue();
    inputs.leftPositionError = leftMotor.getClosedLoopError().getValueAsDouble();
    // Right Logging Values
    inputs.rightTorqueCurrentAmps = rightMotor.getTorqueCurrent().getValueAsDouble();
    inputs.rightPositionRotations =
        Rotation2d.fromRotations(mRightEncoder.getPosition().getValueAsDouble()).getRotations();
    inputs.rightVelocityRotPerSec = rightMotor.getVelocity().getValueAsDouble();
    inputs.rightMotorConnected = rightMotor.isConnected();
    inputs.rightEncoderConnected = mRightEncoder.isConnected();
    inputs.rightControlMode = rightMotor.getControlMode().getValue();
    inputs.rightPositionError = rightMotor.getClosedLoopError().getValueAsDouble();
  }

  public void setDesiredState(ElevatorState state) {
    this.desiredState = state;
    leftMotor.setControl(positionRequest.withPosition(state.pos.getRotations()));
    rightMotor.setControl(positionRequest.withPosition(state.pos.getRotations()));
  }

<<<<<<< HEAD
  public void setPercentOutput(double percent) {
    leftMotor.set(percent);
    rightMotor.set(percent);
  }

  public void stop() {}

  public void hold(double hold) {}
}
=======
  public void setPercentOutput(double percent){
    motor.set(percent);
  }

  public void stop(){

  }
  public void hold(double hold){
    
  }
}
>>>>>>> 9ffdd3ca3a5c258b4be2b0ffa522dfdbd3388ecd
