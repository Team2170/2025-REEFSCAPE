// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.commands.PathPlannerAuto;

import BobcatLib.Hardware.Controllers.OI;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.SwerveDrive;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Commands.ControlledSwerve;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Commands.TeleopSwerve;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.Utility.PIDConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        /* Subsystems */
        public final OI s_Controls;
        public SwerveDrive s_Swerve;
        private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Routine"); // Choose an Auto!
        private final Field2d field;
        private final String robotName;
        
        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer(String robotName) {
                this.robotName = robotName;
                s_Swerve = new SwerveDrive(this.robotName, Robot.isSimulation(), Robot.alliance); // This is the Swerve Library implementation.
                s_Controls = new OI(); // Interfaces with popular controllers and input devices
                // SmartDashboard.putNumber("SpeedLimit", 1);
                initCommand();
                // Register Named Commands
                // NamedCommands.registerCommand("someOtherCommand", new PathPlannerAuto("leaveBase Path"));                        
                field = new Field2d();
                SmartDashboard.putData("Field", field);
                // Configure AutoBuilder last
                configureAutos();
                // Configure the button bindings
                configureButtonBindings();
        }
        
        public void initCommand() {
                DoubleSupplier translation = () -> s_Controls.getTranslationValue();
                DoubleSupplier strafe = () -> s_Controls.getStrafeValue();
                if (!Robot.alliance.isBlueAlliance()) {
                        translation = () -> -s_Controls.getTranslationValue();
                        strafe = () -> -s_Controls.getStrafeValue();
                }
                s_Swerve.setDefaultCommand(
                                new TeleopSwerve(
                                                s_Swerve,
                                                translation,
                                                strafe,
                                                () -> s_Controls.getRotationValue(),
                                                () -> s_Controls.robotCentric.getAsBoolean(),
                                                s_Controls.controllerJson));
        }
        
        public boolean autoChooserInitialized() {
                return autoChooser.get() != null;
        }
                
        /**
         * this should only be called once DS and FMS are attached
         */
        public void configureAutos() {
                /*
                * Auto Chooser
                * 
                * Names must match what is in PathPlanner
                * Please give descriptive names
                */
                PIDConstants tranPid = new PIDConstants(10, 0, 0);// PID constants for translation            
                PIDConstants rotPid = new PIDConstants(7, 0, 0);// PID constants for rotation
                s_Swerve = s_Swerve.withPathPlanner(field, tranPid, rotPid);
                // Configure AutoBuilder last
                autoChooser.addDefaultOption("Do Nothing", Commands.none());
                autoChooser.addOption("Base", new PathPlannerAuto("Base"));
                autoChooser.addOption("Auto1", new PathPlannerAuto("Auto1"));
                autoChooser.addOption("DriveToReef-HP1", new PathPlannerAuto("DriveToReef-HP1"));
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {
                Command zeroGyro = Commands.runOnce(s_Swerve::zeroHeading);
                /* Driver Buttons */
                s_Controls.zeroGyro.onTrue(zeroGyro);
                // Cardinal Modes
                double maxSpeed = s_Swerve.jsonSwerve.chassisSpeedLimits.maxSpeed;
                Command strafeBack = s_Swerve.driveAsCommand(new Translation2d(-1, 0).times(maxSpeed)).repeatedly();
                Command strafeForward = s_Swerve.driveAsCommand(new Translation2d(1, 0).times(maxSpeed)).repeatedly();
                Command strafeLeft = s_Swerve.driveAsCommand(new Translation2d(0, 1).times(maxSpeed)).repeatedly();
                Command strafeRight = s_Swerve.driveAsCommand(new Translation2d(0, -1).times(maxSpeed)).repeatedly();
                s_Controls.dpadForwardBtn.whileTrue(strafeForward);
                s_Controls.dpadBackBtn.whileTrue(strafeBack);
                s_Controls.dpadRightBtn.whileTrue(strafeRight);
                s_Controls.dpadLeftBtn.whileTrue(strafeLeft);
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // This method loads the auto when it is called, however, it is recommended
                // to first load your paths/autos when code starts, then return the
                // pre-loaded auto/path
                String name = autoChooser.get().getName();
                return new PathPlannerAuto(name);
        }

        /**
         * Use this to pass the test command to the main {@link Robot} class.
         * Control pattern is forward, right , backwards, left, rotate in place
         * clockwise, rotate in place counterclowise, forward while rotating Clockwise,
         * forward while rotating counter clockwise
         *
         * @return the command to run in autonomous
         */
        public Command getTestCommand() {
                Command testSwerveForward = new ControlledSwerve(
                                s_Swerve, 0.2, 0.0, 0.0, false, s_Controls.controllerJson).withTimeout(3);
                Command testSwerveRight = new ControlledSwerve(
                                s_Swerve, 0.0, 0.2, 0.0, false, s_Controls.controllerJson).withTimeout(3);
                Command testSwerveBackwards = new ControlledSwerve(
                                s_Swerve, -0.2, 0.0, 0.0, false, s_Controls.controllerJson).withTimeout(3);
                Command testSwerveLeft = new ControlledSwerve(
                                s_Swerve, 0.0, -0.2, 0.0, false, s_Controls.controllerJson).withTimeout(3);
                Command testRIPCW = new ControlledSwerve(s_Swerve, 0.0, 0.0, 0.2, false, s_Controls.controllerJson)
                                .withTimeout(3);
                Command testRIPCCW = new ControlledSwerve(s_Swerve, 0.0, 0.0, -0.2, false, s_Controls.controllerJson)
                                .withTimeout(3);
                Command stopMotorsCmd = new InstantCommand(() -> s_Swerve.stopMotors());
                Command testCommand = testSwerveForward.andThen(testSwerveRight).andThen(testSwerveBackwards)
                                .andThen(testSwerveLeft).andThen(testRIPCW).andThen(testRIPCCW).andThen(stopMotorsCmd);
                return testCommand;
        }
}