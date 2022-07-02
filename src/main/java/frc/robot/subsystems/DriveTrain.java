package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;


public class DriveTrain extends SubsystemBase {
    private final WPI_TalonFX 
        leftMotor1 = new WPI_TalonFX(Constants.left1CANPort), 
        leftMotor2 = new WPI_TalonFX(Constants.left2CANPort), 
        rightMotor1 = new WPI_TalonFX(Constants.right1CANPort), 
        rightMotor2 = new WPI_TalonFX(Constants.right2CANPort);

    
    private final Encoder 
        rightEncoder = new Encoder(Constants.right1CANPort, Constants.right2CANPort, true),
        leftEncoder = new Encoder(Constants.left1CANPort, Constants.left2CANPort, false); //false means not inverted;

    private final Gyro gyro = new ADXRS450_Gyro();

    private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d());;

    DifferentialDrive drive;

    public DriveTrain() {
        //MOTORS
        rightMotor1.setInverted(true);
        rightMotor2.setInverted(true);

        leftMotor1.setNeutralMode(NeutralMode.Brake);
        leftMotor2.setNeutralMode(NeutralMode.Coast);
        rightMotor1.setNeutralMode(NeutralMode.Brake);
        rightMotor2.setNeutralMode(NeutralMode.Coast);

        //taken from Andrew
        leftMotor1.configOpenloopRamp(0.7); // limits acceleration, takes 0.7 seconds to accelerate from 0 to 100%
        leftMotor2.configOpenloopRamp(0.7); // (helps keep robot from rocking around violently every time driver stops)
        rightMotor1.configOpenloopRamp(0.7);
        rightMotor2.configOpenloopRamp(0.7);

        //MotorControllerGroup does not work, below is the way to do it
        leftMotor2.follow(leftMotor1);
        rightMotor2.follow(rightMotor2);

        drive = new DifferentialDrive(leftMotor1, rightMotor1);
        

        //ENCODERS

        //necessary to help wpilib convert encoder counts to distance
        leftEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse);
        rightEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse);
        
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        odometry.update(
            gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
    }
    
    //DRIVE

    //Driving using an xbox controller
    public void manualDrive(double moveSpeed, double rotateSpeed) {
        drive.arcadeDrive(moveSpeed*0.5, rotateSpeed*0.5);
    }

    //Driving using voltages
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMotor1.setVoltage(leftVolts);
        rightMotor2.setVoltage(rightVolts);
        drive.feed();
    }

    public void preventMotorSafetyCheckError() {
        drive.feed();
    }

    //INFORMATIONAL

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    }

    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        leftEncoder.reset();
        rightEncoder.reset();
        odometry.resetPosition(pose, gyro.getRotation2d());
    }

}
