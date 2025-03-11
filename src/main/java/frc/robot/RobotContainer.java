// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
<<<<<<< HEAD
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Constants;
import frc.robot.Subsystems.CoralGrabber.Components.CoralGrabberIOReal;
import frc.robot.Subsystems.CoralGrabber.CoralGrabber;
import frc.robot.Subsystems.Elevator.Components.ElevatorIOReal;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.Utility.ElevatorState;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.drive.GyroIO;
import frc.robot.Subsystems.drive.GyroIOPigeon2;
import frc.robot.Subsystems.drive.ModuleIO;
import frc.robot.Subsystems.drive.ModuleIOSim;
import frc.robot.Subsystems.drive.ModuleIOTalonFX;
import frc.robot.commands.CharacterizationCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
=======
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.AlignOnReef;
import frc.robot.Subsystems.CoralGrabber.CoralGrabber;
import frc.robot.Subsystems.CoralGrabber.Components.CoralGrabberIOReal;
import frc.robot.Subsystems.CoralGrabber.Utility.CoralGrabberConfiguration;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.Components.ElevatorIOReal;
import frc.robot.Subsystems.Elevator.Utility.ElevatorState;
import frc.robot.Subsystems.Limelight.Vision;
import frc.robot.Subsystems.Limelight.VisionIOLimelight;
>>>>>>> 9ffdd3ca3a5c258b4be2b0ffa522dfdbd3388ecd

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
<<<<<<< HEAD
public class RobotContainer {
  // Subsystems
  public final Elevator elevator;
  public final CoralGrabber shooter;
  private final Drive drive;
  //   private final Vision limelight_frontleft;
  //   private final Vision limelight_frontright;
  //   private final Vision limelight_backleft;
  //   private final Vision limelight_backcenter;
  //   private final Vision limelight_backright;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  public final CommandXboxController operator;
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
=======
public class RobotContainer extends SwerveBase {
        public final Elevator elevator;
        public final CoralGrabber shooter;
        // public final Vision limelight_fl;
        // public final Vision limelight_fr;
        public final boolean isSim;

        public final CommandXboxController operator;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer(OI driver_controller,
                        List<LoadablePathPlannerAuto> autos,
                        String robotName,
                        boolean isSim,
                        Alliance alliance,
                        PIDConstants tranPidPathPlanner,
                        PIDConstants rotPidPathPlanner) {

                super(driver_controller, autos, robotName, isSim, alliance, tranPidPathPlanner, rotPidPathPlanner);
                // limelight_fl = new Vision(s_Swerve,new
                // VisionIOLimelight(Constants.LimelightConstants.constants));
                // limelight_fr = new Vision(s_Swerve,new
                // VisionIOLimelight(Constants.LimelightConstants.constants));
                super.s_Swerve.fieldCentric = true;
                this.isSim = isSim;
                elevator = new Elevator(new ElevatorIOReal(Constants.ElevatorConstants.elevatorMasterId,
                                Constants.ElevatorConstants.elevatorFollowerId, Constants.ElevatorConstants.canbus,
                                Constants.ElevatorConstants.elevatorMasterCancoderId));
                shooter = new CoralGrabber("Shooter", new CoralGrabberIOReal(
                                Constants.CoralGrabberConstants.coralGrabberMotorId, "rio", "rio"));
                operator = new CommandXboxController(1);
                configureOperatorButtonBindings();
        }

        public void periodic() {
                // limelight_fl.periodic();
                // limelight_fr.periodic();
                s_Swerve.periodic();
                elevator.periodic();
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        public void configureOperatorButtonBindings() {
                // AutoAlign With Reef
                super.s_Controls.first_controller.getYorTriangle().whileTrue(
                                new AlignOnReef(
                                                super.s_Swerve,
                                                () -> super.s_Controls.getLeftXValue(),
                                                () -> super.s_Controls.getLeftYValue(),
                                                () -> super.s_Controls.getRightXValue(),
                                                () -> super.s_Controls.first_controller.getDPadTriggerRight()
                                                                .getAsBoolean(),
                                                () -> super.s_Controls.first_controller.getDPadTriggerLeft()
                                                                .getAsBoolean()));
                // super.s_Controls.first_controller.getXorSquare().whileTrue(new RunCommand(()
                // -> m_climber.setPercentOut(.5), m_climber)).onFalse(new
                // InstantCommand(m_climber::stop));
                Command holdElevatorPosition = new InstantCommand(() -> elevator.hold(-elevator.HOLD_OUTPUT));
                operator.leftBumper().whileTrue(
                                new InstantCommand(() -> elevator.setPercentOutput(elevator.PERCENT_OUTPUT)))
                                .onFalse(holdElevatorPosition);
                operator.rightBumper().whileTrue(
                                new InstantCommand(() -> elevator.setPercentOutput(-elevator.PERCENT_OUTPUT)))
                                .onFalse(holdElevatorPosition);
                operator.x().whileTrue(
                                        new InstantCommand(() -> elevator.setState(ElevatorState.CORAL_L2)))
                                        .onFalse(holdElevatorPosition);
                operator.a().whileTrue(new InstantCommand(() -> shooter.setIntakeSpeed(-0.30)))
                                .onFalse(new InstantCommand(() -> shooter.setIntakeSpeed(0)));

                operator.b().whileTrue(new InstantCommand(() -> shooter.setIntakeSpeed(0.17)))
                                .onFalse(new InstantCommand(() -> shooter.setIntakeSpeed(0)));
                

        }
>>>>>>> 9ffdd3ca3a5c258b4be2b0ffa522dfdbd3388ecd

        // limelight_frontleft =
        //     new Vision(drive, new VisionIOLimelight(Limelight_FrontLeftConstants.constants));
        // limelight_frontright =
        //     new Vision(drive, new VisionIOLimelight(Limelight_FrontRightConstants.constants));
        // limelight_backcenter =
        //     new Vision(drive, new VisionIOLimelight(Limelight_BackCenterConstants.constants));
        // limelight_backleft =
        //     new Vision(drive, new VisionIOLimelight(Limelight_BackLeftConstants.constants));
        // limelight_backright =
        //     new Vision(drive, new VisionIOLimelight(Limelight_BackRightConstants.constants));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        // limelight_frontleft = new Vision(drive, new VisionIO() {});
        // limelight_frontright = new Vision(drive, new VisionIO() {});
        // limelight_backcenter = new Vision(drive, new VisionIO() {});
        // limelight_backleft = new Vision(drive, new VisionIO() {});
        // limelight_backright = new Vision(drive, new VisionIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        // limelight_frontleft = new Vision(drive, new VisionIO() {});
        // limelight_frontright = new Vision(drive, new VisionIO() {});
        // limelight_backcenter = new Vision(drive, new VisionIO() {});
        // limelight_backleft = new Vision(drive, new VisionIO() {});
        // limelight_backright = new Vision(drive, new VisionIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization",
        CharacterizationCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization",
        CharacterizationCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    elevator =
        new Elevator(
            new ElevatorIOReal(
                Constants.ElevatorConstants.elevatorMasterId,
                Constants.ElevatorConstants.elevatorFollowerId,
                Constants.ElevatorConstants.canbus,
                Constants.ElevatorConstants.elevatorMasterCancoderId));
    shooter =
        new CoralGrabber(
            "Shooter",
            new CoralGrabberIOReal(
                Constants.CoralGrabberConstants.coralGrabberMotorId, "rio", "rio"));
    operator = new CommandXboxController(1);

    // Configure the button bindings
    configureButtonBindings();
  }

  public void periodic() {
    // limelight_fl.periodic();
    // limelight_fr.periodic();
    elevator.periodic();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.JoystickDrive(
            drive,
            () -> controller.getLeftY(),
            () -> controller.getLeftX(),
            () -> -controller.getRightX(),
            controller.a()));

    // Switch to X pattern when X button is pressed
    controller.y().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .a()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    Command holdElevatorPosition = new InstantCommand(() -> elevator.hold(-elevator.HOLD_OUTPUT));
    operator
        .leftBumper()
        .whileTrue(new InstantCommand(() -> elevator.setPercentOutput(elevator.PERCENT_OUTPUT)))
        .onFalse(holdElevatorPosition);
    operator
        .rightBumper()
        .whileTrue(new InstantCommand(() -> elevator.setPercentOutput(-elevator.PERCENT_OUTPUT)))
        .onFalse(holdElevatorPosition);
    operator
        .x()
        .whileTrue(new InstantCommand(() -> elevator.setState(ElevatorState.CORAL_L2)))
        .onFalse(holdElevatorPosition);
    operator
        .a()
        .whileTrue(new InstantCommand(() -> shooter.setIntakeSpeed(-0.30)))
        .onFalse(new InstantCommand(() -> shooter.setIntakeSpeed(0)));

    operator
        .b()
        .whileTrue(new InstantCommand(() -> shooter.setIntakeSpeed(0.17)))
        .onFalse(new InstantCommand(() -> shooter.setIntakeSpeed(0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
