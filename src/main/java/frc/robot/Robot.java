// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.Log;
import frc.robot.utils.frc4048.Logging;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private static int m_periodicCount = 0;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    Log.start();

    // Instantiate our RobotContainer.  This will perform all our button
    // bindings, and put our autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Write the subsystem logging titles.
    Logging.instance().writeAllTitles();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this
   * for items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   * <p>
   * This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Increment the count of periodic calls, wrapping around every 25 calls
    // (equivalent to every 0.5 seconds);
    if(m_periodicCount < 24) {
      m_periodicCount++;
    } else {
      m_periodicCount = 0;
    }

    // Runs the Scheduler.  This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic()
    // methods.  This must be called from the robot's periodic block in order
    // for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Write the subsystem data.
    Logging.instance().writeAllData();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    Log.mode("disabled");
  }

  @Override
  public void disabledPeriodic() {}

  /**
   * This autonomous runs the autonomous command selected by your {@link
   * RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    Log.mode("autonomous");

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    Log.mode("teleop");

    // This makes sure that the autonomous stops running when teleop starts
    // running. If you want the autonomous to continue until interrupted by
    // another command, remove this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    Log.mode("test");

    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {}

  /**
   * This function returns the count of periodic calls. The value wraps around
   * every 25 periodic calls (in other words, every half second).
   */
  public static int getPeriodicCount() {
    return m_periodicCount;
  }
}
