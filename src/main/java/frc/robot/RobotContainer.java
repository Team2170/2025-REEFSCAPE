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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.AlignOnReef;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.Components.ElevatorIOReal;
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
        public final Elevator elevator;
       //public final Vision limelight_fl;
       //public final Vision limelight_fr;
       public final boolean isSim;

//        public final CommandXboxController operator;
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
                //limelight_fl = new Vision(s_Swerve,new VisionIOLimelight(Constants.LimelightConstants.constants));
                //limelight_fr = new Vision(s_Swerve,new VisionIOLimelight(Constants.LimelightConstants.constants));
                super.s_Swerve.fieldCentric = true;
                this.isSim = isSim;
                elevator = new Elevator(new ElevatorIOReal(Constants.ElevatorConstants.elevatorMasterId,Constants.ElevatorConstants.elevatorFollowerId,Constants.ElevatorConstants.canbus,Constants.ElevatorConstants.elevatorMasterCancoderId));
                //operator = new CommandXboxController(1);
        }

        public void periodic() {
                //limelight_fl.periodic();
                //limelight_fr.periodic();
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
                //super.s_Controls.first_controller.getXorSquare().whileTrue(new RunCommand(() -> m_climber.setPercentOut(.5), m_climber)).onFalse(new InstantCommand(m_climber::stop));
                // operator.leftBumper().whileTrue(new InstantCommand(()->elevator.setPercentOutput(elevator.PERCENT_OUTPUT)));
                // operator.rightBumper().whileTrue(new InstantCommand(()->elevator.setPercentOutput(-elevator.PERCENT_OUTPUT)));
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