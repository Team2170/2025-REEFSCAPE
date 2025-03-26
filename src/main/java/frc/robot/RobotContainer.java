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
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.Limelight_FrontLeftConstants;
import frc.robot.Constants.Constants.Limelight_FrontRightConstants;
import frc.robot.Controller.CommandReyannController;
import frc.robot.Subsystems.AlgaeRemover.AlgaeRemover;
import frc.robot.Subsystems.AlgaeRemover.Components.AlgaeRemoverIO;
import frc.robot.Subsystems.AlgaeRemover.Components.AlgaeRemoverIOReal;
import frc.robot.Subsystems.Climber.Climber;
import frc.robot.Subsystems.Climber.Components.ClimberIO;
import frc.robot.Subsystems.Climber.Components.ClimberIOReal;
import frc.robot.Subsystems.CoralGrabber.Components.CoralGrabberIO;
import frc.robot.Subsystems.CoralGrabber.Components.CoralGrabberIOReal;
import frc.robot.Subsystems.CoralGrabber.Components.CoralGrabberIOSim;
import frc.robot.Subsystems.CoralGrabber.CoralGrabber;
import frc.robot.Subsystems.Elevator.Components.ElevatorIO;
import frc.robot.Subsystems.Elevator.Components.ElevatorIOReal;
import frc.robot.Subsystems.Elevator.Components.ElevatorIOSim;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.Utility.ElevatorState;
import frc.robot.Subsystems.Funnel.Components.FunnelIO;
import frc.robot.Subsystems.Funnel.Components.FunnelIOReal;
import frc.robot.Subsystems.Funnel.Funnel;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.drive.GyroIO;
import frc.robot.Subsystems.drive.GyroIOPigeon2;
import frc.robot.Subsystems.drive.ModuleIO;
import frc.robot.Subsystems.drive.ModuleIOSim;
import frc.robot.Subsystems.drive.ModuleIOTalonFX;
import frc.robot.Subsystems.vision.Vision;
import frc.robot.Subsystems.vision.VisionIO;
import frc.robot.Subsystems.vision.VisionIOLimelight;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public final Elevator elevator;
  public final CoralGrabber shooter;
  private final Drive drive;
  private final Climber climber;
  private final Funnel funnel;
  private final AlgaeRemover algaeRemover;
  private final Vision limelight_frontleft;
  private final Vision limelight_frontright;
  // private final Vision limelight_backleft;
  // private final Vision limelight_backcenter;
  // private final Vision limelight_backright;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  public final CommandXboxController operator;

  private final CommandReyannController buttonBoard;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    operator = new CommandXboxController(1);
    buttonBoard = new CommandReyannController(2);

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

        limelight_frontleft =
            new Vision(drive, new VisionIOLimelight(Limelight_FrontLeftConstants.constants));
        limelight_frontright =
            new Vision(drive, new VisionIOLimelight(Limelight_FrontRightConstants.constants));
        // limelight_backcenter =
        // new Vision(drive, new
        // VisionIOLimelight(Limelight_BackCenterConstants.constants));
        // limelight_backleft =
        // new Vision(drive, new
        // VisionIOLimelight(Limelight_BackLeftConstants.constants));
        // limelight_backright =
        // new Vision(drive, new
        // VisionIOLimelight(Limelight_BackRightConstants.constants));
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
        climber = new Climber("climber", new ClimberIOReal());
        funnel = new Funnel("Funnel", new FunnelIOReal());
        algaeRemover = new AlgaeRemover("AlgaeRemover", new AlgaeRemoverIOReal());
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
        limelight_frontleft = new Vision(drive, new VisionIO() {});
        limelight_frontright = new Vision(drive, new VisionIO() {});
        // limelight_backcenter = new Vision(drive, new VisionIO() {});
        // limelight_backleft = new Vision(drive, new VisionIO() {});
        // limelight_backright = new Vision(drive, new VisionIO() {});
        elevator =
            new Elevator(
                new ElevatorIOSim(
                    Constants.ElevatorConstants.elevatorMasterId,
                    Constants.ElevatorConstants.elevatorFollowerId,
                    Constants.ElevatorConstants.canbus,
                    Constants.ElevatorConstants.elevatorMasterCancoderId));
        shooter =
            new CoralGrabber(
                "Shooter",
                new CoralGrabberIOSim(Constants.CoralGrabberConstants.coralGrabberMotorId, "rio"));
        climber = new Climber("climber", new ClimberIO() {});
        funnel = new Funnel("Funnel", new FunnelIO() {});
        algaeRemover = new AlgaeRemover("AlgaeRemover", new AlgaeRemoverIO() {});
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
        limelight_frontleft = new Vision(drive, new VisionIO() {});
        limelight_frontright = new Vision(drive, new VisionIO() {});
        // limelight_backcenter = new Vision(drive, new VisionIO() {});
        // limelight_backleft = new Vision(drive, new VisionIO() {});
        // limelight_backright = new Vision(drive, new VisionIO() {});
        // NamedCommands.registerCommand("flipGyro", new InstantCommand(() ->
        // drive.getPose()));

        elevator = new Elevator(new ElevatorIO() {});
        shooter = new CoralGrabber("Shooter", new CoralGrabberIO() {});
        climber = new Climber("climber", new ClimberIO() {});
        funnel = new Funnel("Funnel", new FunnelIO() {});
        algaeRemover = new AlgaeRemover("AlgaeRemover", new AlgaeRemoverIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    // autoChooser.addOption(
    // "Drive Wheel Radius Characterization",
    // CharacterizationCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    // "Drive Simple FF Characterization",
    // CharacterizationCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    // "Drive SysId (Quasistatic Forward)",
    // drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    // "Drive SysId (Quasistatic Reverse)",
    // drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    // "Drive SysId (Dynamic Forward)",
    // drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    // "Drive SysId (Dynamic Reverse)",
    // drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    NamedCommands.registerCommand(
        "elevate",
        new RepeatCommand(new InstantCommand(() -> elevator.setState(ElevatorState.CORAL_L2)))
            .withTimeout(1));
    NamedCommands.registerCommand(
        "shoot",
        new RepeatCommand(new InstantCommand(() -> shooter.setIntakeSpeed(0.3))).withTimeout(1));

    // Configure the button bindings
    configureButtonBindings();
  }

  public void periodic() {
    limelight_frontleft.periodic();
    limelight_frontright.periodic();
    elevator.periodic();
  }

  public void outputState(String stateName) {
    Logger.recordOutput("Robot/Action", stateName);
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
        DriveCommands.JoystickLimitedDrive(
            drive,
            () -> controller.getLeftY(),
            () -> controller.getLeftX(),
            () -> controller.leftTrigger(0.25).getAsBoolean(),
            () -> -controller.getRightX(),
            controller.b()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // controller
    //     .rightTrigger(0.25)
    //     .whileTrue(
    //         DriveCommands.driveToReef(
    //             drive,
    //             () -> controller.getLeftY(),
    //             () -> controller.getLeftX(),
    //             () -> -controller.getRightX(),
    //             controller.povRight(),
    //             controller.povLeft()));

    // Reset gyro to 0° when B button is pressed
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
        .a()
        .whileTrue(new InstantCommand(() -> shooter.setIntakeSpeed(-0.30)))
        .onFalse(new InstantCommand(() -> shooter.setIntakeSpeed(0)));

    operator
        .b()
        .whileTrue(new InstantCommand(() -> shooter.setIntakeSpeed(0.17)))
        .onFalse(new InstantCommand(() -> shooter.setIntakeSpeed(0)));

    operator
        .y()
        .whileTrue(new InstantCommand(() -> climber.setPercentOut(0.5)))
        .onFalse(new InstantCommand(() -> climber.setPercentOut(0)));

    operator
        .x()
        .whileTrue(new InstantCommand(() -> climber.setPercentOut(-0.25)))
        .onFalse(new InstantCommand(() -> climber.setPercentOut(0)));

    Command stop = new InstantCommand(() -> elevator.stop());
    Command SetpointReached = new RunCommand(() -> elevator.setState(ElevatorState.CORAL_L1));
    Command ScoreLevelOne =
        new SequentialCommandGroup(
            new RunCommand(() -> elevator.setState(ElevatorState.CORAL_L1))
                .alongWith(new InstantCommand(() -> outputState("ElevatorUp")))
                .until(() -> elevator.reachedSetpoint(ElevatorState.CORAL_L1))
                .withTimeout(0.75),
            new RunCommand(() -> shooter.setIntakeSpeed(-0.12))
                .withTimeout(1.5)
                .alongWith(new InstantCommand(() -> outputState("ShooterOutake"))),
            new InstantCommand(() -> shooter.setIntakeSpeed(0)),
            new RunCommand(() -> elevator.setState(ElevatorState.UNKNOWN))
                .alongWith(new InstantCommand(() -> outputState("ElevatorDown")))
                .until(() -> elevator.reachedSetpoint(ElevatorState.UNKNOWN))
                .withTimeout(0.75),
            new InstantCommand(() -> elevator.stop()));
    Command ScoreLevelTwo =
        new SequentialCommandGroup(
            new RunCommand(() -> elevator.setState(ElevatorState.CORAL_L2))
                .alongWith(new InstantCommand(() -> outputState("ElevatorUp")))
                .until(() -> elevator.reachedSetpoint(ElevatorState.CORAL_L2))
                .withTimeout(0.9),
            new RunCommand(() -> shooter.setIntakeSpeed(-0.30))
                .withTimeout(1.5)
                .alongWith(new InstantCommand(() -> outputState("ShooterOutake"))),
            new InstantCommand(() -> shooter.setIntakeSpeed(0)),
            new RunCommand(() -> elevator.setState(ElevatorState.UNKNOWN))
                .alongWith(new InstantCommand(() -> outputState("ElevatorDown")))
                .until(() -> elevator.reachedSetpoint(ElevatorState.UNKNOWN))
                .withTimeout(0.9),
            new InstantCommand(() -> elevator.stop()));
    Command ScoreLevelThree =
        new SequentialCommandGroup(
            new RunCommand(() -> elevator.setState(ElevatorState.CORAL_L3))
                .alongWith(new InstantCommand(() -> outputState("ElevatorUp")))
                .until(() -> elevator.reachedSetpoint(ElevatorState.CORAL_L3))
                .withTimeout(1.75),
            new RunCommand(() -> shooter.setIntakeSpeed(-0.30))
                .withTimeout(1.5)
                .alongWith(new InstantCommand(() -> outputState("ShooterOutake"))),
            new InstantCommand(() -> shooter.setIntakeSpeed(0)),
            new RunCommand(() -> elevator.setState(ElevatorState.UNKNOWN))
                .alongWith(new InstantCommand(() -> outputState("ElevatorDown")))
                .until(() -> elevator.reachedSetpoint(ElevatorState.UNKNOWN))
                .withTimeout(1.75),
            new InstantCommand(() -> elevator.stop()));

    buttonBoard.L1().onTrue(ScoreLevelOne);
    buttonBoard.L2().onTrue(ScoreLevelTwo);
    buttonBoard.L3().onTrue(ScoreLevelThree);

    operator.rightStick().onTrue(new InstantCommand(() -> elevator.resetElevatorPosition()));
    operator
        .povUp()
        .onTrue(new InstantCommand(() -> funnel.setPercentOut(0.1)))
        .onFalse(new InstantCommand(() -> funnel.stop()));

    operator
        .povDown()
        .onTrue(new InstantCommand(() -> funnel.setPercentOut(-0.1)))
        .onFalse(new InstantCommand(() -> funnel.stop()));

    operator
        .povRight()
        .onTrue(new InstantCommand(() -> algaeRemover.setPercentOut(0.4)))
        .onFalse(new InstantCommand(() -> algaeRemover.stop()));

    operator
        .povLeft()
        .onTrue(new InstantCommand(() -> algaeRemover.setPercentOut(-0.4)))
        .onFalse(new InstantCommand(() -> algaeRemover.stop()));

    // controller.y().onTrue(new InstantCommand(() -> drive.flipGyro()));

    // Simulation with keyboard only!!!!
    new CommandJoystick(2)
        .button(5)
        .onTrue(new InstantCommand(() -> elevator.setState(ElevatorState.UNKNOWN)))
        .onFalse(holdElevatorPosition);
    new CommandJoystick(2).button(1).onTrue(ScoreLevelOne);
    new CommandJoystick(2).button(2).onTrue(ScoreLevelTwo);
    new CommandJoystick(2).button(3).onTrue(ScoreLevelThree);
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
