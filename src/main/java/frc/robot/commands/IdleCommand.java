package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DriveTrain;

public class IdleCommand implements Runnable {

    DriveTrain driveTrain;

    public IdleCommand(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
    }

    
    public void initialize() {
        driveTrain.tankDriveVolts(0, 0); //stops robot from moving
    }

    @Override
    public void run() {
            driveTrain.tankDriveVolts(0,0);
    }
    
}
