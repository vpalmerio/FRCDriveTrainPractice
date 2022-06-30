// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static DriveTrain m_driveTrain;

  public static XboxController xbox;

  AHRS ahrs;

  private ParallelCommandGroup teleopCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    xbox = new XboxController(1);

    //ahrs = new AHRS(SPI.Port.kMXP);

    m_driveTrain = new DriveTrain();

    teleopCommand = new ParallelCommandGroup(
      new DriveCommand( ()-> xbox.getLeftY(), ()-> xbox.getLeftX())
    );


  }

  public Command getTeleopCommand() {
    return teleopCommand;
  }

  
}
