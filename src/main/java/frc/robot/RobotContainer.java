// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.CANrange;
import com.fasterxml.jackson.core.TreeNode;

import BobcatLib.Hardware.Controllers.OI;
import BobcatLib.Hardware.Sensors.SpatialSensor.Spatial;
import BobcatLib.Hardware.Sensors.SpatialSensor.SpatialTOF;
import BobcatLib.Hardware.Sensors.SpatialSensor.Components.CANRange;
import BobcatLib.Hardware.Sensors.SpatialSensor.Components.RangeSensor;
import BobcatLib.Hardware.Sensors.SpatialSensor.Components.SENS3006;
import BobcatLib.Hardware.Sensors.SpatialSensor.Components.SimTOF;
import BobcatLib.Hardware.Sensors.SpatialSensor.Utility.DistanceMode;
import BobcatLib.Hardware.Sensors.SpatialSensor.Utility.DistanceMode.modes;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Containers.SwerveBase;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.Utility.PIDConstants;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Utility.Alliance;
import BobcatLib.Subsystems.Swerve.Utility.LoadablePathPlannerAuto;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Commands.SingleTagAlign;
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
        // public final Vision limelight;
        private final Spatial distanceSensing;
        private boolean isSim; 
        private SpatialTOF stof;
        private PIDController alignPID;
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
                // limelight = new Vision(s_Swerve, new VisionIOLimelight(Constants.LimelightConstants.constants));
                
                List<RangeSensor> distanceSensors = new ArrayList<RangeSensor>();
                this.isSim = isSim;

                if (this.isSim) {
                        SimTOF stof = new SimTOF(1, new DistanceMode(modes.SHORT), 20);
                        distanceSensors.add(stof);
                        SimTOF stof1 = new SimTOF(2, new DistanceMode(modes.SHORT), 20);
                        distanceSensors.add(stof1);
                } else {
                        distanceSensors.add(new CANRange(5, new DistanceMode(modes.LONG), 20, "CANt_open_file"));
                        distanceSensors.add(new CANRange(6, new DistanceMode(modes.LONG), 20, "CANt_open_file"));
                }

                stof = new SpatialTOF(distanceSensors);
                distanceSensing = new Spatial(stof);

                alignPID = new PIDController(0.05, 0, 0);
        }

        public void periodic() {
                
                Logger.recordOutput("Sptial/isRunning", false);
                s_Swerve.periodic();
                distanceSensing.periodic();



        }

        public InstantCommand AutolignCommand(){
                return new InstantCommand( () -> AutoAlign());
        }

        public void AutoAlign(){
                Logger.recordOutput("Sptial/isRunning", true);
                double leftRange = distanceSensing.getDistances().get("left");
                double rightRange = distanceSensing.getDistances().get("right");
                if( leftRange > rightRange)
                {
                        // Turn CCW
                        boolean isOpenLoop = true;
                        Translation2d linearTranslation = new Translation2d(0,0);
                        double pidRotation = alignPID.calculate(s_Swerve.getHeading().getDegrees(), s_Swerve.getHeading().getDegrees()+10);
                        s_Swerve.drive(linearTranslation, pidRotation, isOpenLoop,
                                        s_Swerve.getHeading(),
                                        s_Swerve.getPose());
                }
                else if ( rightRange > leftRange)
                {
                        // Turn CW
                        boolean isOpenLoop = true;
                        Translation2d linearTranslation = new Translation2d(0,0);
                        double pidRotation = alignPID.calculate(s_Swerve.getHeading().getDegrees(), s_Swerve.getHeading().getDegrees()-10);                  
                        s_Swerve.drive(linearTranslation, pidRotation, isOpenLoop,
                                        s_Swerve.getHeading(),
                                        s_Swerve.getPose());
                }
                else{

                }
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
                // super.s_Controls.first_controller.getDPadTriggerUp().whileTrue( new SingleTagAlign(s_Swerve,() -> limelight.targetPoseCameraSpace().getX(),()->0 , ()-> Rotation2d.fromDegrees(0)));
                super.s_Controls.first_controller.getRightBumper().whileTrue(AutolignCommand().repeatedly());
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