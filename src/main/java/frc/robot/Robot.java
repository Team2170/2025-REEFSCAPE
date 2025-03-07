// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;

import BobcatLib.BobcatLibCoreRobot;
import BobcatLib.Hardware.Controllers.OI;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.Utility.PIDConstants;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Utility.Alliance;
import BobcatLib.Subsystems.Swerve.Utility.LoadablePathPlannerAuto;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends BobcatLibCoreRobot {
  private Command m_autonomousCommand;

  private Command m_testCommand;
  private OI driver_controller;
  public static Alliance alliance;

  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    super(RobotBase.isReal());
    alliance = new Alliance();
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    List<LoadablePathPlannerAuto> loadableAutos = new ArrayList<LoadablePathPlannerAuto>();
    loadableAutos.add(new LoadablePathPlannerAuto("Do Nothing", Commands.none(), true));
    String robotName = "2024_Robot";
    boolean isSim = false;
    PIDConstants tranPidPathPlanner = new PIDConstants(10, 0, 0);
    PIDConstants rotPidPathPlanner = new PIDConstants(5, 0, 0);
    driver_controller = new OI(robotName);
    m_robotContainer = new RobotContainer(driver_controller, loadableAutos, robotName,
    isSim, alliance, tranPidPathPlanner,
    rotPidPathPlanner);
    loadableAutos.add(new LoadablePathPlannerAuto("Base", new PathPlannerAuto("Base").withName("Base"), false));
    loadableAutos.add(new LoadablePathPlannerAuto("Auto1", new PathPlannerAuto("Auto1").withName("Auto1"), false));
    m_robotContainer.updateLoadedPaths(loadableAutos);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    String name  = "";
    name = m_robotContainer.getAutoChooser().getSendableChooser().getSelected();
    if( name == "" || name == null){
      name  = m_robotContainer.getAutoChooser().get().getName();
    }
    m_autonomousCommand = m_robotContainer.getAutonomousCommand(name);

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    m_testCommand = m_robotContainer.getTestCommand();
    m_testCommand.schedule();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

  }
}
