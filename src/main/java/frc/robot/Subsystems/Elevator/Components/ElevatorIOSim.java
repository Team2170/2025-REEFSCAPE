package frc.robot.Subsystems.Elevator.Components;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.*;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.Constants;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.Utility.ElevatorState;

public class ElevatorIOSim implements ElevatorIO {

  private DCMotor gearBox = DCMotor.getKrakenX60Foc(2);

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
  final PositionDutyCycle positionRequest = new PositionDutyCycle(0);

  private ElevatorState desiredState = ElevatorState.UNKNOWN;

  public double leftOffset = 0; // In Rotations
  public double rightOffset = 0; // In Rotations
  private final TalonFXSimState leftSim;
  private final TalonFXSimState rightSim;
  private final CANcoderSimState leftSensSim;
  private final CANcoderSimState rightSensSim;

  public double leftPos = 0.0;
  public double rightPos = 0.0;
  public double leftVel = 0.0;
  public double rightVel = 0.0;

  // State given by elevator carriage position and velocity
  // Input given by torque current to motor
  private Vector<N2> simState;

  private boolean isClosedLoop = false;

  public static final double kElevatorKp = 5;
  public static final double kElevatorKi = 0;
  public static final double kElevatorKd = 0;
  public static final double kElevatorkS = 0.0; // volts (V)
  public static final double kElevatorkG = 0.762; // volts (V)
  public static final double kElevatorkV = 0.762; // volt per velocity (V/(m/s))
  public static final double kElevatorkA = 0.0; // volt per acceleration (V/(m/s²))
  public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
  public static final double kCarriageMass = 4.0; // kg

  // This gearbox represents a gearbox containing 2 Kraken Motors
  private final DCMotor m_elevatorGearbox = DCMotor.getKrakenX60Foc(2);

  // Standard classes for controlling our elevator
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(
          kElevatorKp, kElevatorKi, kElevatorKd, new TrapezoidProfile.Constraints(2.45, 2.45));
  // Simulation classes help us simulate what's going on, including gravity.
  ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(kElevatorkS, kElevatorkG, kElevatorkV, kElevatorkA);
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          m_elevatorGearbox,
          GEAR_RATIO,
          kCarriageMass,
          kElevatorDrumRadius,
          0,
          Elevator.ELEVATOR_MAX_HEIGHT.baseUnitMagnitude(),
          true,
          0,
          0.01,
          0.0);

  public ElevatorIOSim(int motorID, int motorFollowerId, String canbus, int encoderID) {
    leftMotor = new TalonFX(motorID, canbus);
    rightMotor = new TalonFX(motorFollowerId, canbus);
    mLeftEncoder = new CANcoder(encoderID, canbus);
    mRightEncoder = new CANcoder(encoderID + 1, canbus);

    leftSim = leftMotor.getSimState();
    rightSim = rightMotor.getSimState();
    leftSensSim = mLeftEncoder.getSimState();
    rightSensSim = mRightEncoder.getSimState();

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    rightMotor.getConfigurator().apply(motorConfig); // reset to factory default
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = 120;
    motorConfig.Feedback.RotorToSensorRatio = GEAR_RATIO;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.Slot0.kP = 1;
    motorConfig.Slot0.kI = 0;
    motorConfig.Slot0.kD = 0;
    motorConfig.Slot0.kS = 0;
    motorConfig.Slot0.kG = 0;
    /* Open and Closed Loop Ramping */
    motorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
    motorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;
    motorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.0;
    motorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
    leftMotor.getConfigurator().apply(motorConfig);
    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = 120;
    motorConfig.Feedback.RotorToSensorRatio = GEAR_RATIO;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfig.Slot0.kP = 1;
    motorConfig.Slot0.kI = 0;
    motorConfig.Slot0.kD = 0;
    motorConfig.Slot0.kS = 0;
    motorConfig.Slot0.kG = 0;
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
  }

  private Angle metersToRotations(Distance meters) {
    Distance kWheelRadius = Inches.of(3);
    /* Divide the distance by the wheel radius to get radians */
    var wheelRadians = meters.in(Meters) / kWheelRadius.in(Meters);
    /* Then multiply by gear ratio to get rotor rotations */
    return Radians.of(wheelRadians * GEAR_RATIO);
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    /* Pass the robot battery voltage to the simulated devices */
    leftSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    leftSensSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    rightSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    rightSensSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    leftMotor.setControl(positionRequest.withPosition(desiredState.pos.getRotations()));
    rightMotor.setControl(positionRequest.withPosition(desiredState.pos.getRotations()));

    double leftTorque = leftSim.getTorqueCurrent();
    double rightTorque = rightSim.getTorqueCurrent();
    if (desiredState == ElevatorState.UNKNOWN) {
      leftPos = 0;
      leftVel = MetersPerSecond.of(0).baseUnitMagnitude();
      leftTorque = 0;
      rightTorque = leftTorque;
    } else {
      leftVel = MetersPerSecond.of(1).baseUnitMagnitude();
      if (leftPos < desiredState.pos.getRotations()) {
        leftPos += 1;
      } else {
        leftPos -= 1;
      }
    }

    rightPos = leftPos;
    rightVel = leftVel;

    leftSensSim.setRawPosition(leftPos);
    leftSensSim.setVelocity(leftVel);
    rightSensSim.setRawPosition(rightPos);
    rightSensSim.setVelocity(rightVel);
    leftSim.setRawRotorPosition(leftPos);
    leftSim.setRotorVelocity(leftVel);
    rightSim.setRawRotorPosition(rightPos);
    rightSim.setRotorVelocity(rightVel);

    inputs.state = desiredState;
    inputs.targetRotations = desiredState.pos.getRotations();
    // Left Logging Values
    inputs.leftTorqueCurrentAmps = leftTorque;
    inputs.leftPositionRotations = leftPos;
    inputs.leftVelocityRotPerSec = leftVel;
    inputs.leftMotorConnected = leftMotor.isConnected();
    inputs.leftEncoderConnected = mLeftEncoder.isConnected();
    inputs.leftControlMode = leftMotor.getControlMode().getValue();
    inputs.leftPositionError =
        leftMotor.getClosedLoopError().getValueAsDouble() + (desiredState.pos.getRotations() * -2);
    // Right Logging Values
    inputs.rightTorqueCurrentAmps = rightTorque;
    inputs.rightPositionRotations = rightPos;
    inputs.rightVelocityRotPerSec = rightVel;
    inputs.rightMotorConnected = rightMotor.isConnected();
    inputs.rightEncoderConnected = mRightEncoder.isConnected();
    inputs.rightControlMode = rightMotor.getControlMode().getValue();
    inputs.rightPositionError = inputs.leftPositionError;
  }

  public void setDesiredState(ElevatorState state) {
    isClosedLoop = true;
    this.desiredState = state;
  }

  public void setPercentOutput(double percent) {
    isClosedLoop = false;
    leftMotor.set(percent);
    rightMotor.set(percent);
  }

  public void stop() {
    leftMotor.set(0);
    rightMotor.set(0);
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
