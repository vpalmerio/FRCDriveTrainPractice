package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;


public class DriveTrain extends SubsystemBase {
    private WPI_VictorSPX m_left1, m_left2, m_right1, m_right2;

    MotorControllerGroup m_left;
    MotorControllerGroup m_right;

    DifferentialDrive drive;

    public DriveTrain() {
        m_left1 = new WPI_VictorSPX(Constants.m_left1);
        m_left2 = new WPI_VictorSPX(Constants.m_left2);
        m_right1 = new WPI_VictorSPX(Constants.m_right1);
        m_right2 = new WPI_VictorSPX(Constants.m_right2);

        m_right1.setInverted(true);
        m_right2.setInverted(true);

        m_left1.setNeutralMode(NeutralMode.Coast);
        m_left2.setNeutralMode(NeutralMode.Coast);
        m_right1.setNeutralMode(NeutralMode.Coast);
        m_right2.setNeutralMode(NeutralMode.Coast);

        //taken from Andrew
        m_left1.configOpenloopRamp(0.7); // limits acceleration, takes 0.4 seconds to accelerate from 0 to 100%
        m_left2.configOpenloopRamp(0.7); // (helps keep robot from rocking around violently every time driver stops)
        m_right1.configOpenloopRamp(0.7);
        m_right2.configOpenloopRamp(0.7);

        //you can also create motor controller group
        m_left2.follow(m_left1);
        m_right2.follow(m_right2);

        drive = new DifferentialDrive(m_left1, m_right1);     
        
    }
    
    public void manualDrive(double moveSpeed, double rotateSpeed) {
        drive.arcadeDrive(moveSpeed, rotateSpeed);
    }


}
