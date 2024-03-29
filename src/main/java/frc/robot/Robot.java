// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


//Load Path
import java.nio.file.Path;
import java.sql.Driver;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;

//Neccessary stuff
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Command m_teleopCommand;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    
    

    //m_autonomousCommand = m_robotContainer.getExampleAutonomousCommand();
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
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


  private static final String quickPathTrajectoryJSON = "paths/QuickPath.wpilib.json";

  private static final Field2d m_field = new Field2d();

  @Override
  public void disabledPeriodic() {

    /*
    try {
      //eventually use hashes to check if the previous hash is different from the new hash, then load the new trajectory
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(quickPathTrajectoryJSON);
      Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      
      SmartDashboard.putData(m_field);
      m_field.getObject("traj").setTrajectory(trajectory);
    } catch (IOException ex) {
      DriverStation.reportWarning("Unable to open trajectory: " + quickPathTrajectoryJSON, ex.getStackTrace());
    }
    */

  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    //String trajectoryJSON = Constants.quickPathTrajectoryJSON;
    String trajectoryJSON = Constants.circleTrajectoryJSON;

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      m_autonomousCommand = m_robotContainer.loadAutonomousCommand(trajectory);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
      m_autonomousCommand = null;
    }

    m_robotContainer.resetOdometryAutonomous();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if(m_robotContainer.collisionDetection()) {
        m_autonomousCommand.cancel();
        disabledInit();
    }
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

    m_teleopCommand = m_robotContainer.getTeleopCommand();

    // Schedule teleop command
    if(m_teleopCommand != null) {
      m_teleopCommand.schedule();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
