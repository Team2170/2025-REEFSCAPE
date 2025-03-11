package frc.robot.Subsystems.Elevator.Components;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
<<<<<<< HEAD
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.Constants;
=======
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

>>>>>>> 9ffdd3ca3a5c258b4be2b0ffa522dfdbd3388ecd
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.Utility.ElevatorState;

public class ElevatorIOReal implements ElevatorIO {

  public static final double GEAR_RATIO = 12.00; // SET THIS UP
<<<<<<< HEAD
  public static final InvertedValue ELEVATOR_MOTOR_INVERTED =
      InvertedValue.CounterClockwise_Positive; // SET THIS UP
=======
  public static final InvertedValue ELEVATOR_MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive; // SET THIS UP
>>>>>>> 9ffdd3ca3a5c258b4be2b0ffa522dfdbd3388ecd

  /**
   * note that kG is different from ks, even they are both static forces, ks always opposes the
   * direction of motion, kg is always in the same direction, regardless of which way the elevator
   * is moving
   */
<<<<<<< HEAD
  private TalonFX leftMotor;
=======
  private TalonFX motor;
  private TalonFX mFollower;
  private CANcoder mEncoder;
>>>>>>> 9ffdd3ca3a5c258b4be2b0ffa522dfdbd3388ecd

  private TalonFX rightMotor;
  private CANcoder mLeftEncoder;
  private CANcoder mRightEncoder;

  // create a Motion Magic request, voltage output
  final PositionDutyCycle positionRequest = new PositionDutyCycle(0);

  private ElevatorState desiredState = ElevatorState.UNKNOWN;

  public ElevatorIOReal(int motorID, int motorFollowerId, String canbus, int encoderID) {
    leftMotor = new TalonFX(motorID, canbus);
    rightMotor = new TalonFX(motorFollowerId, canbus);
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
<<<<<<< HEAD
    rightMotor.getConfigurator().apply(motorConfig); // reset to factory default
=======
    motor.getConfigurator().apply(motorConfig); // reset to factory default
>>>>>>> 9ffdd3ca3a5c258b4be2b0ffa522dfdbd3388ecd
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = 120;
    motorConfig.Feedback.RotorToSensorRatio = GEAR_RATIO;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
<<<<<<< HEAD
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Elevator.MAX_ROTATIONS.getRotations();
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
=======
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Elevator.MAX_ROTATIONS.getRotations();
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

>>>>>>> 9ffdd3ca3a5c258b4be2b0ffa522dfdbd3388ecd
    motorConfig.Slot0.kP = 1;
    motorConfig.Slot0.kI = 0;
    motorConfig.Slot0.kD = 0;
    motorConfig.Slot0.kS = 0;
    motorConfig.Slot0.kG = 0;
<<<<<<< HEAD
    leftMotor.getConfigurator().apply(motorConfig);
=======
>>>>>>> 9ffdd3ca3a5c258b4be2b0ffa522dfdbd3388ecd

    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = 120;
    motorConfig.Feedback.RotorToSensorRatio = GEAR_RATIO;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        -Elevator.MAX_ROTATIONS.getRotations();
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.Slot0.kP = 1;
    motorConfig.Slot0.kI = 0;
    motorConfig.Slot0.kD = 0;
    motorConfig.Slot0.kS = 0;
    motorConfig.Slot0.kG = 0;
    rightMotor.getConfigurator().apply(motorConfig);

<<<<<<< HEAD
    mLeftEncoder = new CANcoder(Constants.ElevatorConstants.elevatorMasterCancoderId);
    CANcoderConfiguration leftEncoderConfig = new CANcoderConfiguration();
    leftEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    leftEncoderConfig.MagnetSensor.MagnetOffset = 0.20874; // TODO find this
    mLeftEncoder.getConfigurator().apply(leftEncoderConfig);

    mRightEncoder = new CANcoder(Constants.ElevatorConstants.elevatorFollowerCancoderId);
    CANcoderConfiguration rightEncoderConfig = new CANcoderConfiguration();
    rightEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    rightEncoderConfig.MagnetSensor.MagnetOffset = 0.20874; // TODO find this
    mRightEncoder.getConfigurator().apply(rightEncoderConfig);
  }

  public void updateInputs(ElevatorIOInputs inputs) {
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
=======
    mFollower = new TalonFX(motorFollowerId, canbus);
    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = 120;
    motorConfig.Feedback.RotorToSensorRatio = GEAR_RATIO;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = -Elevator.MAX_ROTATIONS.getRotations();
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;


    motorConfig.Slot0.kP = 1;
    motorConfig.Slot0.kI = 0;
    motorConfig.Slot0.kD = 0;
    motorConfig.Slot0.kS = 0;
    motorConfig.Slot0.kG = 0;
    
    motor.getConfigurator().apply(motorConfig);

  }

  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.torqueCurrentAmps = motor.getTorqueCurrent().getValueAsDouble();
    inputs.velocityRotPerSec = motor.getVelocity().getValueAsDouble();
    // inputs.rotPosition =
    // Rotation2d.fromRotations(encoder.getPosition().getValueAsDouble());
    // inputs.positionPercent = encoder.getPosition().getValueAsDouble() /
    // Elevator.MAX_ROTATIONS.getRotations();
    // inputs.aligned = Math.abs(encoder.getPosition().getValueAsDouble()
    // - desiredState.pos.getRotations()) <
    // Elevator.ELEVATOR_TOLERANCE.getRotations();
    inputs.motorConnected = motor.isConnected();
    // inputs.encoderConnected = encoder.isConnected();
    inputs.controlMode = motor.getControlMode().getValue();
    inputs.positionRotations = inputs.rotPosition.getRotations();
    inputs.state = desiredState;
    inputs.heightMeters = inputs.positionRotations * Elevator.METERS_PER_ROTATION;

    Logger.recordOutput("Elevator/leftPosition", motor.getPosition().getValueAsDouble());
    Logger.recordOutput("Elevator/rightPosition", mFollower.getPosition().getValueAsDouble());
>>>>>>> 9ffdd3ca3a5c258b4be2b0ffa522dfdbd3388ecd
  }

  public void setDesiredState(ElevatorState state) {
    this.desiredState = state;
<<<<<<< HEAD
    leftMotor.setControl(positionRequest.withPosition(state.pos.getRotations()));
    rightMotor.setControl(positionRequest.withPosition(state.pos.getRotations()));
=======
    motor.setControl(positionRequest.withPosition(state.pos.getRotations()));
    mFollower.setControl(positionRequest.withPosition(state.pos.getRotations()));
  }

  public void setPercentOutput(double percent) {
    motor.set(percent);
    mFollower.set(percent);
  }

  public void stop() {
    motor.stopMotor();
    mFollower.stopMotor();
  }

  public void hold(double hold) {
    setPercentOutput(hold);
>>>>>>> 9ffdd3ca3a5c258b4be2b0ffa522dfdbd3388ecd
  }

  public void setPercentOutput(double percent) {
    leftMotor.set(percent);
    rightMotor.set(percent);
  }

  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public void hold(double hold) {
    setPercentOutput(hold);
  }
}
