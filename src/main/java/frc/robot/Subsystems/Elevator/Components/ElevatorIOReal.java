package frc.robot.Subsystems.Elevator.Components;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.Utility.ElevatorState;

public class ElevatorIOReal implements ElevatorIO {

  public static final double GEAR_RATIO = 12.00; // SET THIS UP
  public static final InvertedValue ELEVATOR_MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive; // SET THIS UP

  /**
   * note that kG is different from ks, even they are both static forces, ks
   * always opposes the
   * direction of motion, kg is always in the same direction, regardless of which
   * way the elevator
   * is moving
   */
  private TalonFX motor;
  private TalonFX mFollower;

  private CANcoder encoder;

  private MotionMagicTorqueCurrentFOC positionRequest = new MotionMagicTorqueCurrentFOC(0);

  private ElevatorState desiredState = ElevatorState.UNKNOWN;

  public ElevatorIOReal(int motorID, int motorFollowerId, String canbus, int encoderID) {
    motor = new TalonFX(motorID, canbus);
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motor.getConfigurator().apply(motorConfig); // reset to factory default
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = 120;
    motorConfig.Feedback.RotorToSensorRatio = GEAR_RATIO;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Elevator.MAX_ROTATIONS.getRotations();
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motor.getConfigurator().apply(motorConfig);

    mFollower = new TalonFX(motorFollowerId, canbus);
    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = 120;
    motorConfig.Feedback.RotorToSensorRatio = GEAR_RATIO;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = -Elevator.MAX_ROTATIONS.getRotations();
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motor.getConfigurator().apply(motorConfig);
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.torqueCurrentAmps = motor.getTorqueCurrent().getValueAsDouble();
    inputs.velocityRotPerSec = motor.getVelocity().getValueAsDouble();
    // inputs.rotPosition = Rotation2d.fromRotations(encoder.getPosition().getValueAsDouble());
    // inputs.positionPercent = encoder.getPosition().getValueAsDouble() / Elevator.MAX_ROTATIONS.getRotations();
    // inputs.aligned = Math.abs(encoder.getPosition().getValueAsDouble()
    //     - desiredState.pos.getRotations()) < Elevator.ELEVATOR_TOLERANCE.getRotations();
    inputs.motorConnected = motor.isConnected();
    //inputs.encoderConnected = encoder.isConnected();
    inputs.controlMode = motor.getControlMode().getValue();
    inputs.positionRotations = inputs.rotPosition.getRotations();
    inputs.state = desiredState;
    inputs.heightMeters = inputs.positionRotations * Elevator.METERS_PER_ROTATION;

    Logger.recordOutput("Elevator/leftPosition", motor.getPosition().getValueAsDouble());
    Logger.recordOutput("Elevator/rightPosition", mFollower.getPosition().getValueAsDouble());
  }

  public void setDesiredState(ElevatorState state) {
    this.desiredState = state;
    motor.setControl(positionRequest.withPosition(state.pos.getRotations()));
    mFollower.setControl(positionRequest.withPosition(state.pos.getRotations()));
  }

  public void setPercentOutput(double percent){
    motor.set(percent);
    mFollower.set(percent);
  }

  public void stop(){
    motor.stopMotor();
    mFollower.stopMotor();
  }

  public void hold(double hold){
    setPercentOutput(hold);
  }
}