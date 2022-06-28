package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;


public class DriveTrain extends SubsystemBase {
    private VictorSPX m_left1, m_left2, m_right1, m_right2;

    MotorControllerGroup m_left;
    MotorControllerGroup m_right;

    DifferentialDrive drive;

    DriveTrain() {
        m_left1 = new VictorSPX(Constants.m_left1);
        m_left2 = new VictorSPX(Constants.m_left2);
        m_right1 = new VictorSPX(Constants.m_right1);
        m_right2 = new VictorSPX(Constants.m_right2);

        m_left2.follow(m_left1);
        m_right2.follow(m_right2);

        

    }
   

}
