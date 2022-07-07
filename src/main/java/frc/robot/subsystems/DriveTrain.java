package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;


public class DriveTrain extends SubsystemBase {
    private final WPI_TalonFX 
        leftMotor1 = new WPI_TalonFX(Constants.left1CANPort), 
        leftMotor2 = new WPI_TalonFX(Constants.left2CANPort), 
        rightMotor1 = new WPI_TalonFX(Constants.right1CANPort), 
        rightMotor2 = new WPI_TalonFX(Constants.right2CANPort);

    boolean left_side_inverted = Constants.left_side_inverted;
    boolean right_side_inverted = Constants.right_side_inverted;

    private final Gyro gyro = new ADXRS450_Gyro();

    private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

    DifferentialDrive drive;

    public DriveTrain() {
        //MOTORS

        

        leftMotor1.setInverted(left_side_inverted);
        leftMotor2.setInverted(InvertType.FollowMaster);

        rightMotor1.setInverted(right_side_inverted);
        rightMotor2.setInverted(InvertType.FollowMaster);
          
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
        rightMotor2.follow(rightMotor1);

        drive = new DifferentialDrive(leftMotor1, rightMotor1);
        
        

        //ENCODERS (with help from https://docs.ctre-phoenix.com/en/stable/ch14_MCSensor.html)
        // and here: https://docs.wpilib.org/en/stable/docs/hardware/sensors/encoders-hardware.html?highlight=encoder#on-shaft-encoders


        //encoders are automatically inverted when motor controller is inverted, according to phoenix doc
        leftMotor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        leftMotor2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        rightMotor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        rightMotor2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);        


        resetEncoders();
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block

        var gyroAngle = Rotation2d.fromDegrees(gyro.getAngle());

        //for get distance, multiply distance per pulse by position/pulses/units of sensor 
        //(position of sensor should overflow, as in not go back to zero once it goes from 2048-2049, with 2048 being the CPR of the integrated encoder)
        double leftEncoderPosition = Constants.kDistancePerEncoderCycle*leftMotor1.getSelectedSensorPosition();
        double rightEncoderPosition = Constants.kDistancePerEncoderCycle*rightMotor1.getSelectedSensorPosition();

        odometry.update(
            gyroAngle, leftEncoderPosition, rightEncoderPosition);

        SmartDashboard.putNumber("Left Encoder Position", leftEncoderPosition);
        SmartDashboard.putNumber("Right Encoder Position", rightEncoderPosition);

    }
    
    //DRIVE

    //Driving using an xbox controller
    public void manualDrive(double moveSpeed, double rotateSpeed) {
        drive.arcadeDrive(moveSpeed*0.5, rotateSpeed*0.5);
    }

    //Driving using voltages
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMotor1.setVoltage(leftVolts);
        rightMotor1.setVoltage(rightVolts);
        drive.feed();
    }

    //SIMULATION
    //@Override
    //public void simulationPeriodic() {
    //    TalonFXSimCollection leftMotor1Sim = leftMotor1.getSimCollection();
    //    leftMotor1Sim.setIntegratedSensorRawPosition((int) leftMotor1.getSelectedSensorPosition());
    //}

    //INFORMATIONAL

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {

        //getSelectedSensorVelocity gives pulses per 100ms
        //convert to pules per second (multiply)
        //multiply result by Constants.kDistancePerEncoderCycle
            //^^AKA, multiplying pulses per second by Distance (meters) per pulse to get Distance (meters) per second
                                                                        //Convert from 100ms to 1 second
        double leftEncoderVelocity = Constants.kDistancePerEncoderCycle*(leftMotor1.getSelectedSensorVelocity()*10); 

        double rightEncoderVelocity = Constants.kDistancePerEncoderCycle*(leftMotor1.getSelectedSensorVelocity()*10); 
        
        

        return new DifferentialDriveWheelSpeeds(leftEncoderVelocity, rightEncoderVelocity);
    }

    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetEncoders() {
        rightMotor1.setSelectedSensorPosition(0);
        leftMotor1.setSelectedSensorPosition(0);
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, gyro.getRotation2d());
    }

}
