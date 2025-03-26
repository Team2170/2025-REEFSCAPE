package frc.robot.Subsystems.Elevator.Components;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.Constants;
import frc.robot.Subsystems.Elevator.Utility.ElevatorState;

public class ElevatorIOReal implements ElevatorIO {

  public static final double GEAR_RATIO = 12.00; // SET THIS UP
  public static final InvertedValue ELEVATOR_MOTOR_INVERTED =
      InvertedValue.CounterClockwise_Positive; // SET THIS UP

  /**
   * note that kG is different from ks, even they are both static forces, ks always opposes the
   * direction of motion, kg is always in the same direction, regardless of which way the elevator
   * is moving
   */
  private TalonFX leftMotor;

  private TalonFX rightMotor;
  private CANcoder mLeftEncoder;
  private CANcoder mRightEncoder;

  // create a Motion Magic request, voltage output
  final PositionDutyCycle positionRequest = new PositionDutyCycle(0).withVelocity(2);

  private ElevatorState desiredState = ElevatorState.UNKNOWN;

  public double leftOffset = -0.4296; // In Rotations
  public double rightOffset = -0.140869; // In Rotations

  public ElevatorIOReal(int motorID, int motorFollowerId, String canbus, int encoderID) {
    leftMotor = new TalonFX(motorID, canbus);
    rightMotor = new TalonFX(motorFollowerId, canbus);
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    rightMotor.getConfigurator().apply(motorConfig); // reset to factory default
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = 35;
    motorConfig.Feedback.RotorToSensorRatio = GEAR_RATIO;
    motorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    motorConfig.Slot0.kP = 0.5;
    motorConfig.Slot0.kI = 0;
    motorConfig.Slot0.kD = 0;
    motorConfig.Slot0.kS = 0;
    motorConfig.Slot0.kG = 0.5;

    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -60;

    motorConfig.MotorOutput.PeakForwardDutyCycle = 0.3;
    /* Open and Closed Loop Ramping */
    motorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
    motorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;
    motorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.0;
    motorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
    leftMotor.getConfigurator().apply(motorConfig);
    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = 35;
    motorConfig.Feedback.RotorToSensorRatio = GEAR_RATIO;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfig.Slot0.kP = 0.5;
    motorConfig.Slot0.kI = 0;
    motorConfig.Slot0.kD = 0;
    motorConfig.Slot0.kS = 0;
    motorConfig.Slot0.kG = 0.5;
    motorConfig.MotorOutput.PeakForwardDutyCycle = 0.3;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -60;
    motorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
    motorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;
    motorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.0;
    motorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
    rightMotor.getConfigurator().apply(motorConfig);

    mLeftEncoder = new CANcoder(Constants.ElevatorConstants.elevatorMasterCancoderId);
    CANcoderConfiguration leftEncoderConfig = new CANcoderConfiguration();
    leftEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    leftEncoderConfig.MagnetSensor.MagnetOffset = leftOffset; // TODO find this
    mLeftEncoder.getConfigurator().apply(leftEncoderConfig);

    mRightEncoder = new CANcoder(Constants.ElevatorConstants.elevatorFollowerCancoderId);
    CANcoderConfiguration rightEncoderConfig = new CANcoderConfiguration();
    rightEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    rightEncoderConfig.MagnetSensor.MagnetOffset = rightOffset; // TODO find this
    mRightEncoder.getConfigurator().apply(rightEncoderConfig);

    // Set the internal encoder to the absolute encoder.
    rightMotor.setPosition(mRightEncoder.getPosition().getValueAsDouble());
    leftMotor.setPosition(mLeftEncoder.getPosition().getValueAsDouble());
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.state = desiredState;
    inputs.targetRotations = desiredState.pos.getRotations();
    // Left Logging Values
    inputs.leftTorqueCurrentAmps = leftMotor.getTorqueCurrent().getValueAsDouble();
    inputs.leftPositionRotations =
        Rotation2d.fromRotations(mLeftEncoder.getPosition().getValueAsDouble() - leftOffset)
            .getRotations();
    inputs.leftVelocityRotPerSec = leftMotor.getVelocity().getValueAsDouble();
    inputs.leftMotorConnected = leftMotor.isConnected();
    inputs.leftEncoderConnected = mLeftEncoder.isConnected();
    inputs.leftControlMode = leftMotor.getControlMode().getValue();
    inputs.leftPositionError = leftMotor.getClosedLoopError().getValueAsDouble();
    // Right Logging Values
    inputs.rightTorqueCurrentAmps = rightMotor.getTorqueCurrent().getValueAsDouble();
    inputs.rightPositionRotations =
        Rotation2d.fromRotations(mRightEncoder.getPosition().getValueAsDouble() - rightOffset)
            .getRotations();
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

  public boolean reachedSetpoint(ElevatorState state) {
    double threshold = Rotation2d.fromRotations(1).getRotations();
    double leftRotations =
        absRotation2d(
                Rotation2d.fromRotations(
                    mLeftEncoder.getPosition().getValueAsDouble() - leftOffset))
            .getRotations();
    double rightRotations =
        absRotation2d(
                Rotation2d.fromRotations(
                    mRightEncoder.getPosition().getValueAsDouble() - rightOffset))
            .getRotations();
    double targetRotations = absRotation2d(state.pos).getRotations();
    if (isWithinThreshold(leftRotations, targetRotations, threshold)) {
      return true;
    }
    if (isWithinThreshold(rightRotations, targetRotations, threshold)) {
      return true;
    }
    return false;
  }

  public Rotation2d absRotation2d(Rotation2d rot) {
    Rotation2d absRot = Rotation2d.fromRotations(Math.abs(rot.getRotations()));
    return absRot;
  }

  public boolean isWithinThreshold(double value, double target, double threshold) {
    return Math.abs(value - target) <= threshold;
  }
}
