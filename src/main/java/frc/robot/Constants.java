package frc.robot;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.PivotRange;
import frc.lib.Swerve.util.COTSFalconSwerveConstants;
import frc.lib.Swerve.util.SwerveModuleConstants;
import frc.robot.subsystems.Climber.module.ClimberModuleConstants;
import frc.robot.subsystems.Intake.module.IntakeModuleConstants;
import frc.robot.subsystems.Vision.module.VisionModuleConstants;
import frc.robot.subsystems.Vision.module.VisionModuleType;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import static edu.wpi.first.apriltag.AprilTagFields.k2024Crescendo;

public final class Constants {
        public static final Mode currentMode = RobotBase.isSimulation() ? Mode.SIM
                        : (RobotBase.isReal() ? Mode.REAL : Mode.REPLAY);

        public static enum Mode {
                /** Running on a real robot. */
                REAL,

                /** Running a physics simulator. */
                SIM,

                /** Replaying from a log file. */
                REPLAY
        }

        public static final double stickDeadband = 0.1;
        public static final String canivorename = "Crescendo"; // TODO: CHANGE NAME TO MATCH ROBOT

        public static final class FieldConstants {
                public static final double fieldLength = 16.541; // meters
                public static final double fieldWidth = 8.211;

                // 1 is closest to AMP, 5 is closest to SOURCE
                public static final Translation2d centerlineNote1 = new Translation2d(250.5, 29.64);
                public static final Translation2d centerlineNote2 = new Translation2d(250.5, 95.64);
                public static final Translation2d centerlineNote3 = new Translation2d(250.5, 161.64);
                public static final Translation2d centerlineNote4 = new Translation2d(250.5, 227.64);
                public static final Translation2d centerlineNote5 = new Translation2d(250.5, 293.64);

                public static final double noteDiameter = Units.inchesToMeters(14);

                public static final double speakerHeight = Units.inchesToMeters(80.4375); // Center of opening

                public static final Translation2d blueSpeakerPose = new Translation2d(Units.inchesToMeters(-1.5),
                                Units.inchesToMeters(218.42)); // Center of back of the opening
                public static final Translation2d redSpeakerPose = new Translation2d(Units.inchesToMeters(652.73),
                                Units.inchesToMeters(218.42)); // Center of back of the opening //652.73

        }

        public static final class AprilTagConstants {
                public static AprilTagFieldLayout layout;
                static {
                        try {
                                layout = AprilTagFieldLayout.loadFromResource(k2024Crescendo.m_resourceFile);
                        } catch (Exception e) {
                                e.printStackTrace();
                        }
                }
        }

        public static final class ShooterConstants {
                public static final int shooterID = 36;
                public static final int followerID = 31;

                public static final class FieldConstants {
                        public static final double speakerHeight = Units.inchesToMeters(80.4375); // Center of opening
                }

                /* Module Gear Ratios */
                public static final double flywheelGearRatio = 40 / 1;
                /* flywheel Motor info */
                public static final double kFreeSpeedRpm = 5676;
                public static final double kflywheelWheelFreeSpeedRps = kFreeSpeedRpm / 60;
                /* flywheel Motor PID Values */
                public static final double flywheelKP = 0.005;
                public static final double flywheelKI = 0.0;
                public static final double flywheelKD = 0.0;
                public static final double flywheelKFF = 1 / kflywheelWheelFreeSpeedRps;
                // Max Output Powers
                public static final double flywheelPower = 1;

                /* Flywheel Current Limiting */
                public static final int flywheelContinuousCurrentLimit = 35;
                /* Motor Characteristics */
                public static final boolean leftFlywheelMotorInvert = true;
                public static final boolean rightFlywheelMotorInvert = false;
                public static final CANSparkFlex.IdleMode flywheelIdleMode = CANSparkMax.IdleMode.kCoast;
                // meters per rotation
                public static final double wheelCircumference = Units.inchesToMeters(0.5);
                public static final double flywheelRevToMeters = wheelCircumference / (flywheelGearRatio);
                public static final double flywheelRpmToMetersPerSecond = flywheelRevToMeters / 60;
        }

        public final class ClimberConstants {
                // Spark Max Idle Modes
                public static final CANSparkMax.IdleMode climberIdleMode = CANSparkMax.IdleMode.kBrake;
                // Spark Flex Invert Modes
                public static final boolean leftInvert = true;
                public static final boolean rightInvert = false;
                // Max Output Powers
                public static final double climberPower = 1;
                /* Angle Encoder Invert */
                public static final boolean canCoderInvert = false;
                /* Motor Inverts */
                /* Climber Current Limiting */
                public static final int climberContinuousCurrentLimit = 40;
                public static final int climberPeakCurrentLimit = 50;
                public static final double climberPeakCurrentDuration = 0.1;
                public static final boolean climberEnableCurrentLimit = true;
                /* Drive Motor info */
                public static final double kFreeSpeedRpm = 5676; // Per minute
                public static final double kDriveWheelFreeSpeedRps = kFreeSpeedRpm / 60; // per second
                /* Climber Motor PID Values */
                public static final double climberKP = 0.04;
                public static final double climberKI = 0.0;
                public static final double climberKD = 0.0;
                public static final double climberKFF = 1 / kDriveWheelFreeSpeedRps;
                /** Meters per Second */
                public static final double maxSpeed = 3.6576;
                /** Radians per Second */
                public static final double climberRampRate = 0;

                // Climber PID
                public static final double kP = 0.1;
                public static final double kI = 1e-4;
                public static final double kD = 1;
                public static final double kIz = 0;
                public static final double kFF = 0;

                public static final class leftClimber {
                        public static final double angleOffset = 0;
                        public static final int climberId = 35;
                        public static final boolean isInverted = false;
                        public static final ClimberModuleConstants config = new ClimberModuleConstants(angleOffset,
                                        climberId, isInverted);

                        public class config {
                        }
                }

                public static final class rightClimber {
                        public static final double angleOffset = 0;
                        public static final int climberId = 32;
                        public static final boolean isInverted = true;
                        public static final ClimberModuleConstants config = new ClimberModuleConstants(angleOffset,
                                        climberId, isInverted);

                        public class config {
                        }
                }
        }

        public static final class IntakeConstants {

                // Spark Max Idle Modes
                public static final CANSparkMax.IdleMode driveIdleMode = CANSparkMax.IdleMode.kBrake;
                public static final CANSparkMax.IdleMode angleIdleMode = CANSparkMax.IdleMode.kBrake;

                // Max Output Powers
                public static final double drivePower = 1;
                public static final double anglePower = .9;
                /* Motor Inverts */
                public static final boolean isRollerInverted = false;
                public static final boolean isPivotInverted = false;
                // encoder setup
                public static final double angleOffset = 0.0;
                // meters per rotation
                public static final double wheelCircumference = Units.inchesToMeters(0.5); // tune this as the total
                                                                                           // circumpherence of hte
                                                                                           // intake?
                /* Intake Current Limiting */
                public static final int pivotContinuousCurrentLimit = 20;
                public static final int pivotPeakCurrentLimit = 40;
                public static final boolean pivotEnableCurrentLimit = true;
                public static final int rollerContinuousCurrentLimit = 35;
                public static final int rollerPeakCurrentLimit = 40;
                public static final boolean rollerEnableCurrentLimit = true;

                /* Pivot Motor PID Values */
                public static final int pivotId = 33;
                public static final double angleKP = 0.0001;
                public static final double angleKI = 0;
                public static final double angleKD = 0;
                public static final double angleKFF = 0;

                /* Rollers Motor info */
                public static final int rollerId = 34;
                public static final double kFreeSpeedRpm = 6784;// from documentation RPM
                public static final double kRollerWheelFreeSpeedRps = 6784 / 60; // RPS

                /* Drive Motor PID Values */
                public static final double driveKP = 0.04;
                public static final double driveKI = 0.0;
                public static final double driveKD = 0.0;
                public static final double driveKFF = 1 / kRollerWheelFreeSpeedRps;
                /** Radians per Second */
                public static double angleRampRate = 0;

                public static PivotRange pivotAngleRange = new PivotRange(180, 30);

                // CanCoder
                public static final int absEncoderId = 17;
                public static final SensorDirectionValue sensorDirection = SensorDirectionValue.CounterClockwise_Positive;
                public static final Rotation2d offset = new Rotation2d();
                public static final AbsoluteSensorRangeValue sensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;

                // Module Config!!
                public static final IntakeModuleConstants moduleConstants = new IntakeModuleConstants(0, pivotId,
                                rollerId, absEncoderId, false,
                                false);
        }

        public static final class SwerveConstants {
                public static final int pigeonID = 35;
                public static final COTSFalconSwerveConstants chosenModule = // TODO: This must be tuned to specific
                                                                             // robot
                                COTSFalconSwerveConstants.SDS.MK4
                                                .KrakenX60(COTSFalconSwerveConstants.SDS.MK4.driveRatios.L2);

                /* Drivetrain Constants */
                public static final double trackWidth = Units.inchesToMeters(24); // TODO: This must be tuned to
                                                                                  // specific robot
                public static final double wheelBase = Units.inchesToMeters(24); // TODO: This must be tuned to specific
                                                                                 // robot
                public static final double wheelCircumference = chosenModule.wheelCircumference;

                /*
                 * Swerve Kinematics
                 * No need to ever change this unless you are not doing a traditional
                 * rectangular/square 4 module swerve
                 */
                public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

                /* Module Gear Ratios */
                public static final double driveGearRatio = chosenModule.driveGearRatio;
                public static final double angleGearRatio = chosenModule.angleGearRatio;

                /* Motor Inverts */
                public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
                public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

                /* Angle Encoder Invert */
                public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

                /* Swerve Current Limiting */
                public static final int angleCurrentLimit = 25;
                public static final int angleCurrentThreshold = 40;
                public static final double angleCurrentThresholdTime = 0.1;
                public static final boolean angleEnableCurrentLimit = true;
                public static final int driveCurrentLimit = 35;
                public static final int driveCurrentThreshold = 60;
                public static final double driveCurrentThresholdTime = 0.1;
                public static final boolean driveEnableCurrentLimit = true;

                /*
                 * These values are used by the drive falcon to ramp in open loop and closed
                 * loop driving.
                 * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
                 */
                public static final double openLoopRamp = 0.25;
                public static final double closedLoopRamp = 0.0;

                /* Angle Motor PID Values */
                public static final double angleKP = chosenModule.angleKP;
                public static final double angleKI = chosenModule.angleKI;
                public static final double angleKD = chosenModule.angleKD;

                /* Drive Motor PID Values */
                public static final double driveKP = 0.05; // TODO: This must be tuned to specific robot
                public static final double driveKI = 0.0;
                public static final double driveKD = 0.0;
                public static final double driveKF = 0.0;

                /*
                 * Drive Motor Characterization Values From SYSID
                 * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-
                 * identification/introduction.html
                 */
                public static final double driveKS = 0.32; // TODO: This must be tuned to specific robot
                public static final double driveKV = 1.51;
                public static final double driveKA = 0.27;

                /* Swerve Profiling Values */
                /** Meters per Second */
                public static final double maxSpeed = 4.5; // TODO: This must be tuned to specific robot
                /** Radians per Second */
                public static final double maxAngularVelocity = 10.0; // TODO: This must be tuned to specific robot
                /** Meters per Second */
                public static final double maxAccel = 3;
                /** Meters per Second */
                public static final double maxAngularAcceleration = Math.PI / 2;

                /* Neutral Modes */
                public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
                public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

                /* Module Specific Constants */
                /* Front Left Module - Module 0 */
                public static final class Mod0 { // TODO: This must be tuned to specific robot
                        public static final int driveMotorID = 10;
                        public static final int angleMotorID = 11;
                        public static final int canCoderID = 20;
                        public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.156);
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID,
                                        canCoderID, angleOffset);
                }

                /* Front Right Module - Module 1 */
                public static final class Mod1 { // TODO: This must be tuned to specific robot
                        public static final int driveMotorID = 14;
                        public static final int angleMotorID = 15;
                        public static final int canCoderID = 22;
                        public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.120);
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID,
                                        canCoderID, angleOffset);
                }

                /* Back Left Module - Module 2 */
                public static final class Mod2 { // TODO: This must be tuned to specific robot
                        public static final int driveMotorID = 12;
                        public static final int angleMotorID = 13;
                        public static final int canCoderID = 21;
                        public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.384);
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID,
                                        canCoderID, angleOffset);
                }

                /* Back Right Module - Module 3 */
                public static final class Mod3 { // TODO: This must be tuned to specific robot
                        public static final int driveMotorID = 16;
                        public static final int angleMotorID = 17;
                        public static final int canCoderID = 23;
                        public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.034);
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID,
                                        canCoderID, angleOffset);
                }
        }

        public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be
                                                  // tuned to specific robot
                public static final double kMaxSpeedMetersPerSecond = 3;
                public static final double kMaxAccelerationMetersPerSecondSquared = 3;
                public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
                public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

                public static final double kPXController = 1;
                public static final double kPYController = 1;
                public static final double kPThetaController = 1;

                /* Constraint for the motion profilied robot angle controller */
                public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
        }

        public static final class VisionConstants {

                public static class leftShooter {
                        public static String name = "limelight-left";
                        public static double verticalFOV;
                        public static double horizontalFOV;
                        public static double limelightMountHeight;
                        public static VisionModuleType moduleType;
                        public static int pipelineIndex;
                        public static int horPixels;
                        public static int vertPixels;
                        public static double filterTimeConstant = 0.1;
                        public static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.1,0.1, Units.degreesToRadians(10));
                        public static int movingAverageNumTaps = 20;
                        public static String ip = "10.21.70.13";
                        public static VisionModuleConstants config = new VisionModuleConstants(name,
                                        verticalFOV, horizontalFOV, limelightMountHeight, moduleType, pipelineIndex,
                                        horPixels, vertPixels, filterTimeConstant, visionMeasurementStdDevs,
                                        movingAverageNumTaps);
                }

                public static class rightShooter {
                        public static String name = "limelight-right";
                        public static double verticalFOV;
                        public static double horizontalFOV;
                        public static double limelightMountHeight;
                        public static VisionModuleType moduleType;
                        public static int pipelineIndex;
                        public static int horPixels;
                        public static int vertPixels;
                        public static double filterTimeConstant = 0.1;
                        public static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.1,0.1, Units.degreesToRadians(10));
                        public static int movingAverageNumTaps = 20;
                        public static String ip = "10.21.70.12";
                        public static VisionModuleConstants config = new VisionModuleConstants(name,
                                        verticalFOV, horizontalFOV, limelightMountHeight, moduleType, pipelineIndex,
                                        horPixels, vertPixels, filterTimeConstant, visionMeasurementStdDevs,
                                        movingAverageNumTaps);
                }

                public static class objectDetector {
                        public static String name = "limelight-note";
                        public static double verticalFOV;
                        public static double horizontalFOV;
                        public static double limelightMountHeight;
                        public static VisionModuleType moduleType;
                        public static int pipelineIndex = 1;
                        public static int horPixels = 1280;
                        public static int vertPixels;
                        public static double filterTimeConstant = 0.1;
                        public static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.1,0.1, Units.degreesToRadians(10));
                        public static int movingAverageNumTaps = 20;
                        public static String ip = "10.21.70.11";
                        public static VisionModuleConstants config = new VisionModuleConstants(name,
                                        verticalFOV, horizontalFOV, limelightMountHeight, moduleType, pipelineIndex,
                                        horPixels, vertPixels, filterTimeConstant, visionMeasurementStdDevs,
                                        movingAverageNumTaps);
                }

                public static final class LimelightConstants {
                        public static final double verticalFOV = 49.7; // degrees obviously
                        public static final double horizontalFOV = 63.3; // degrees obviously
                        public static final double limelightMountHeight = Units.inchesToMeters(20);
                        public static final int detectorAprilTagPipelineIndex = 2;
                        public static final int detectorNotePipelineIndex = 1;
                        public static final int horPixles = 1280;
                        public static final double filterTimeConstant = 0.1; // in seconds, inputs occuring over a time period
                                                                             // significantly shorter than this will be thrown out
                
                    }
        }
}