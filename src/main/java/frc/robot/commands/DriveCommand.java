package frc.robot.commands;

import frc.robot.RobotContainer;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveCommand extends CommandBase {
    
    private DoubleSupplier moveSpeed;
    private DoubleSupplier rotateSpeed;

    //this double supplier object usage is necessary to get the robot to move
    //I still don't know why
    public DriveCommand(DoubleSupplier moveSpeed, DoubleSupplier rotateSpeed) {

        this.moveSpeed = moveSpeed;
        this.rotateSpeed = rotateSpeed;        

        addRequirements(RobotContainer.m_driveTrain);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        RobotContainer.m_driveTrain.manualDrive(moveSpeed.getAsDouble(), rotateSpeed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
