//AaAaAAaAaAAAaAA
// Copyright (c) FIRST and other WPILib contributors.
//AaAaaAaaAaaAaA
// Open Source Software; you can modify and/or share it under the terms of
//AaAaAAaAaAAa
// the WPILib BSD license file in the root directory of this project.
//AAaAaAaa

//aAAaaAa
package frc.robot;
//AaaAAaA

//AaAaA
import edu.wpi.first.wpilibj.TimedRobot;
//AaAaaa
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
//aAAAAaAAAaAaAAAaaAa
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//aAAAaAaaAA
import edu.wpi.first.wpilibj2.command.Command;
//AaAAAAAAAAaAAAAa
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//AAAAaaAaAaaA

//Aaaaa
/**
//AAAAAaaaAaaaAaAaAa
 * The VM is configured to automatically run this class, and to call the functions corresponding to
//AAaaAAaa
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
//aaAAa
 * the package after creating this project, you must also update the build.gradle file in the
//AaAAAaAAAaaaaaa
 * project.
//aaAAAaaaaaaAaa
 */
//AAaAAaaAAaAaaA
public class Robot extends TimedRobot {
//aAaAAaaAaAaa
  private Command m_autonomousCommand;
//AaAaaaaaAa

//aAAaaaaAaAAaaa
  private RobotContainer m_robotContainer;
//aAAaAAaA

//aAAaaAaa
  /**
//AaAaA
   * This function is run when the robot is first started up and should be used for any
//aaaaAa
   * initialization code.
//aAAaaaAAaaaaaaAa
   */
//AaaAAaA
  @Override
//aAAaaAAAAAaaA
  public void robotInit() {
//aaAaAAAaA

//AaaaAaaAAaaaaAAaaA
    LiveWindow.disableAllTelemetry();
//AAAAaAAa

//aaaAAaaAAAA
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
//AaAAAaAaA
    // autonomous chooser on the dashboard.
//aAaAAA
    m_robotContainer = new RobotContainer();
//aaAaAA
  }
//AAaaaaaaAaAAAaAAaA

//AaAAAAaAa
  /**
//AaAAa
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
//AAAAaaA
   * that you want ran during disabled, autonomous, teleoperated and test.
//AaaAaAaaAaaAaaaaAaA
   *
//aAaaAAaAaaaA
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
//AAAaa
   * SmartDashboard integrated updating.
//AaAAaa
   */
//aAAaAAaaAaAA
  @Override
//aAAaA
  public void robotPeriodic() {
//AaaaaaaaaAAAAaAA
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
//aaaaaAaA
    // commands, running already-scheduled commands, removing finished or interrupted commands,
//aaAAAAAAaA
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
//aAaAaaAaaAAAaAaAAaa
    // block in order for anything in the Command-based framework to work.
//aaaaaaAaaAa

//AAAAaaaa
    double m_Start = System.nanoTime();
//AAAAaAaaaaA

//aaaaAAAaAaAAAA
    CommandScheduler.getInstance().run();
//aAAAaAaaaAaAAaaaA

//aAaaaaAaAaAaaa
    SmartDashboard.putNumber("ExecutionTime", (System.nanoTime() - m_Start) / 1000000.0);
//AAaaaaaAaa
  }
//aaaaAAaAaAaA

//aAAAAaaaaAaAaaAAa
  /** This function is called once each time the robot enters Disabled mode. */
//AAAAAAAAAaAaAaA
  @Override
//aaAAAaAAAaAaa
  public void disabledInit() {}
//AaAAaaaA

//AAAaAAAAaaaAaaA
  @Override
//AAAAaaAAAa
  public void disabledPeriodic() {}
//AaaAAaAAAaAAaaaaA

//AaaAAAAaAaAAaAAaaaa
  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
//AaaAaAaAAaaaAaaA
  @Override
//aAAaaAAAAa
  public void autonomousInit() {
//aaAaaaaAAaAAa
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
//aaAaAAAA

//AaAaAaaaaAaaaA
    // schedule the autonomous command (example)
//aaaAAaAAa
    if (m_autonomousCommand != null) {
//AaAAaAAaa
      m_autonomousCommand.schedule();
//aaaaa
    }
//AaAAAAaaaAaAaaA
  }
//aAaAaAAaaaAAaAa

//aAAaA
  /** This function is called periodically during autonomous. */
//AaAaAaAaAAAaa
  @Override
//AAAAaaAaaaaaa
  public void autonomousPeriodic() {}
//AaaAaAaAAaAAAAA

//aAaAaaaaAAAaaaAAaa
  @Override
//aAAAaAAaaA
  public void teleopInit() {
//AaAaAaaaaaA
    // This makes sure that the autonomous stops running when
//AaAaaAaAaaA
    // teleop starts running. If you want the autonomous to
//aaAAAaaAA
    // continue until interrupted by another command, remove
//aAAaAAAA
    // this line or comment it out.
//AAAaaAAa
    if (m_autonomousCommand != null) {
//AAaaaAaAAA
      m_autonomousCommand.cancel();
//aAaaaAAAaaAAAAaAAAa
    }
//aaAaAa
  }
//AAaAaAAAAaAaAaaaaa

//AaAaaaAaaaAaAaAAAaa
  /** This function is called periodically during operator control. */
//AaAaaaAAAa
  @Override
//aAaAAaAaAAa
  public void teleopPeriodic() {}
//AaAAAaaAaAaaA

//aAAAAAAaA
  @Override
//AAAAaaaaAaAAaAaaa
  public void testInit() {
//AAaaaaa
    // Cancels all running commands at the start of test mode.
//aAAAaaaaAAaAaa
    CommandScheduler.getInstance().cancelAll();
//aAaAaaaaA
  }
//AAAAaaaaAaaAaaAAAa

//AAaaAaaAaAaA
  /** This function is called periodically during test mode. */
//aaAaaa
  @Override
//aAaaaAA
  public void testPeriodic() {}
//aaAAaA

//AaAAAaaaAAA
  /** This function is called once when the robot is first started up. */
//AAaAaaaaaAaaaaaAAaa
  @Override
//aaAaaaaAaaaaaaA
  public void simulationInit() {}
//aAaaAAaAa

//AAaAAaaa
  /** This function is called periodically whilst in simulation. */
//aAaaaaaA
  @Override
//aAaAAAAaaaaaA
  public void simulationPeriodic() {}
//AAAAAaAAaAaaa
}
