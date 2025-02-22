// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import BobcatLib.Hardware.Controllers.OI;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Containers.SwerveBase;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.Utility.PIDConstants;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Utility.Alliance;
import BobcatLib.Subsystems.Swerve.Utility.LoadablePathPlannerAuto;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.AlignOnReef;
import frc.robot.Commands.AlignToSurface;
import frc.robot.Subsystems.Limelight.Vision;
import frc.robot.Subsystems.Limelight.VisionIOLimelight;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer extends SwerveBase {
       public final Vision limelight;
       public final boolean isSim;
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
                limelight = new Vision(s_Swerve,new VisionIOLimelight(Constants.LimelightConstants.constants));
                super.s_Swerve.fieldCentric = true;
                this.isSim = isSim;
        }

        public void periodic() {
                limelight.periodic();
                s_Swerve.periodic();
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        @Override
        public void configureButtonBindings() {
                super.configureButtonBindings();
                // AutoAlign With Reef
                super.s_Controls.first_controller.getYorTriangle().whileTrue(
                        new AlignOnReef(
                        super.s_Swerve,
                        ()->super.s_Controls.getLeftXValue(),
                        ()->super.s_Controls.getLeftYValue(),
                        ()->super.s_Controls.getRightXValue(),
                        ()->super.s_Controls.first_controller.getDPadTriggerRight().getAsBoolean(),
                        ()->super.s_Controls.first_controller.getDPadTriggerLeft().getAsBoolean()));     
                super.s_Controls.first_controller.getXorSquare().whileTrue(
                        new AlignToSurface(
                        super.s_Swerve,
                        ()->super.s_Controls.getLeftXValue(),
                        ()->super.s_Controls.getLeftYValue(),
                        ()->super.s_Controls.getRightXValue(),
                        ()->super.s_Controls.first_controller.getDPadTriggerRight().getAsBoolean(),
                        ()->super.s_Controls.first_controller.getDPadTriggerLeft().getAsBoolean(),super.s_Controls.controllerJson, isSim));     
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        @Override
        public Command getAutonomousCommand(String name) {
                // This method loads the auto when it is called, however, it is recommended
                // to first load your paths/autos when code starts, then return the
                // pre-loaded auto/path
                return super.getAutonomousCommand(name);
        }

        /**
         * Use this to pass the test command to the main {@link Robot} class.
         * Control pattern is forward, right , backwards, left, rotate in place
         * clockwise, rotate in place counterclowise, forward while rotating Clockwise,
         * forward while rotating counter clockwise
         *
         * @return the command to run in autonomous
         */
        @Override
        public Command getTestCommand() {
                return super.getTestCommand();
        }
}