package frc.robot.commands;

import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveCommand extends CommandBase {

    //this double supplier object usage is necessary to get the robot to move
    //I still don't know why
    public DriveCommand() {      

        addRequirements(RobotContainer.driveTrain);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        RobotContainer.driveTrain.manualDrive(
            RobotContainer.xbox.getLeftY(), RobotContainer.xbox.getRightX()
        );
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
