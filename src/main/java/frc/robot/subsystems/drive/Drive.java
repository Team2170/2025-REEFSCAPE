package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants;

public class Drive extends SubsystemBase {
    static final double ODOMETRY_FREQUENCY = new CANBus(DriveConstants.DrivetrainConstants.CANBusName).isNetworkFD()
            ? 250.0
            : 100.0;

    private Rotation2d gyroRot = new Rotation2d();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[] {
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
            };

    private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          90,
          6.883,
          new ModuleConfig(
              DriveConstants.FrontLeft.WheelRadius,
              DriveConstants.kSpeedAt12Volts.in(MetersPerSecond),
              1.2,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(DriveConstants.FrontLeft.DriveMotorGearRatio),
              DriveConstants.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    static final Lock odometryLock = new ReentrantLock();

    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
            new SwerveDriveKinematics(getModuleTranslations()),
            gyroRot,
            lastModulePositions,
            new Pose2d());

    public Drive() {
        modules[0] = new Module(new ModuleIOTalonFX(DriveConstants.FrontLeft), 0, DriveConstants.FrontLeft);
        modules[1] = new Module(new ModuleIOTalonFX(DriveConstants.FrontRight), 1, DriveConstants.FrontRight);
        modules[2] = new Module(new ModuleIOTalonFX(DriveConstants.BackLeft), 2, DriveConstants.BackLeft);
        modules[3] = new Module(new ModuleIOTalonFX(DriveConstants.BackRight), 3, DriveConstants.BackRight);

        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds,
                this::runVelocity,
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                PP_CONFIG, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose() {

    }

    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
                new Translation2d(DriveConstants.FrontLeft.LocationX, DriveConstants.FrontLeft.LocationY),
                new Translation2d(DriveConstants.FrontRight.LocationX, DriveConstants.FrontRight.LocationY),
                new Translation2d(DriveConstants.BackLeft.LocationX, DriveConstants.BackLeft.LocationY),
                new Translation2d(DriveConstants.BackRight.LocationX, DriveConstants.BackRight.LocationY)
        };
    }

    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    private ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.kSpeedAt12Volts);

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    @Override
    public void periodic() {

    }
}