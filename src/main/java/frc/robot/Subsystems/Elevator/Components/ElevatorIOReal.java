package frc.robot.Subsystems.Elevator.Components;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.Utility.ElevatorState;

public class ElevatorIOReal implements ElevatorIO {

  public static final double GEAR_RATIO = 0.00; // SET THIS UP
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

    motorConfig.MotorOutput.Inverted = ELEVATOR_MOTOR_INVERTED;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = 120;

    motorConfig.Slot0.kP = 1;
    motorConfig.Slot0.kI = 0;
    motorConfig.Slot0.kD = 0;
    motorConfig.Slot0.kS = 0;
    motorConfig.Slot0.kG = 0;
    motorConfig.MotionMagic.MotionMagicAcceleration = 4.5;
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = 7.695;
    motorConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

    motorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    motorConfig.Feedback.FeedbackRemoteSensorID = encoderID;
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    motorConfig.Feedback.RotorToSensorRatio = GEAR_RATIO;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Elevator.MAX_ROTATIONS.getRotations();
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    motor.getConfigurator().apply(motorConfig);

    encoder = new CANcoder(encoderID);
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoder.getConfigurator().apply(encoderConfig);
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig.MagnetSensor.MagnetOffset = 0.219971; // TODO find this
    encoder.getConfigurator().apply(encoderConfig);

    mFollower = new TalonFX(motorFollowerId, canbus);
    mFollower.getConfigurator().apply(motorConfig);
    mFollower.setControl(new Follower(motor.getDeviceID(), false));
  }

  public void updateInputs(ElevatorIOInputs inputs) {
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
}