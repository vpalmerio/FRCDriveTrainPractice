package frc.robot.subsystems;

//Falcon Motors
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

//Driving and info
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;

//Gyro
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

//Misc
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Simulation
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;

//Constants
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
    private final WPI_TalonFX 
        leftMotor1 = new WPI_TalonFX(Constants.left1CANPort), 
        leftMotor2 = new WPI_TalonFX(Constants.left2CANPort), 
        rightMotor1 = new WPI_TalonFX(Constants.right1CANPort), 
        rightMotor2 = new WPI_TalonFX(Constants.right2CANPort);

    boolean left_side_inverted = Constants.left_side_inverted;
    boolean right_side_inverted = Constants.right_side_inverted;

    //private final Gyro gyro = new ADXRS450_Gyro();

    //navX gyro
    private final AHRS gyro = new AHRS(SPI.Port.kMXP); 

    private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

    DifferentialDrive drive;

    //Encoders
    int window_size = 1;
    SensorVelocityMeasPeriod measurement_period = SensorVelocityMeasPeriod.Period_1Ms;

    //Collision Detection
    double last_world_linear_accel_x = 0;
    double last_world_linear_accel_y = 0;
    final static double kCollisionThreshold_DeltaG = 2;

    public DriveTrain() {
        //MOTORS
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

        leftMotor1.setInverted(left_side_inverted);
        leftMotor2.setInverted(InvertType.FollowMaster);

        rightMotor1.setInverted(right_side_inverted);
        rightMotor2.setInverted(InvertType.FollowMaster);

        drive = new DifferentialDrive(leftMotor1, rightMotor1);
        
        

        //ENCODERS (with help from https://docs.ctre-phoenix.com/en/stable/ch14_MCSensor.html)
        // and here: https://docs.wpilib.org/en/stable/docs/hardware/sensors/encoders-hardware.html?highlight=encoder#on-shaft-encoders


        //encoders are automatically inverted when motor controller is inverted, according to phoenix doc
        leftMotor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        leftMotor2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        rightMotor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        rightMotor2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);   

        
        //Velocity Measurement period of 1 millisecond and rolling measurement window of 1 (sample)
        //apparently this is the best for accuracy
        //https://www.chiefdelphi.com/t/limiting-voltage-pathweaver-and-ramesetecommand/405377/14

        leftMotor1.configVelocityMeasurementPeriod(measurement_period);
        leftMotor1.configVelocityMeasurementWindow(window_size);
        leftMotor2.configVelocityMeasurementPeriod(measurement_period);
        leftMotor2.configVelocityMeasurementWindow(window_size);
        rightMotor1.configVelocityMeasurementPeriod(measurement_period);
        rightMotor1.configVelocityMeasurementWindow(window_size);
        rightMotor2.configVelocityMeasurementPeriod(measurement_period);
        rightMotor2.configVelocityMeasurementWindow(window_size);
        

        resetEncoders();
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block

        var gyroAngle = gyro.getRotation2d();
        gyro.getDisplacementX();

        //for get distance, multiply distance per pulse by counts/units of sensor 
        //(position of sensor should overflow, as in not go back to zero once it goes from 2048-2049, with 2048 being the CPR of the integrated encoder)
        double leftEncoderPosition = Constants.kDistancePerEncoderCount*leftMotor1.getSelectedSensorPosition();
        double rightEncoderPosition = Constants.kDistancePerEncoderCount*rightMotor1.getSelectedSensorPosition();

        odometry.update(gyroAngle, leftEncoderPosition, rightEncoderPosition);
        
        

        SmartDashboard.putNumber("Left Encoder Position", leftEncoderPosition);
        SmartDashboard.putNumber("Right Encoder Position", rightEncoderPosition);
        SmartDashboard.putNumber("Gyro Heading", gyro.getAngle());

        var translation = odometry.getPoseMeters().getTranslation();
        SmartDashboard.putNumber("X Position", translation.getX());
        SmartDashboard.putNumber("Y Position", translation.getY());

    }

    public boolean collisionDetection() {
        
        //taken from https://pdocs.kauailabs.com/navx-mxp/examples/collision-detection/
        double curr_world_linear_accel_x = gyro.getWorldLinearAccelX();
        double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
        last_world_linear_accel_x = curr_world_linear_accel_x;
        double curr_world_linear_accel_y = gyro.getWorldLinearAccelY();
        double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
        last_world_linear_accel_y = curr_world_linear_accel_y;
        
        if ( ( Math.abs(currentJerkX) > kCollisionThreshold_DeltaG ) || ( Math.abs(currentJerkY) > kCollisionThreshold_DeltaG) ) 
        {
            SmartDashboard.putBoolean("Collision", true);
            return true;
        } else {
            return false;
        }
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
    //https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20General/DifferentialDrive_Simulation/src/main/java/frc/robot/Robot.java
    //^^Use this in the future
    //@Override
    //public void simulationPeriodic() {
    //    TalonFXSimCollection leftMotor1Sim = leftMotor1.getSimCollection();
    //    leftMotor1Sim.setIntegratedSensorRawPosition((int) leftMotor1.getSelectedSensorPosition());
    //}

    //INFORMATIONAL

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {

        //getSelectedSensorVelocity gives pulses/counts per 100ms
        //convert to pules per second (multiply)
        //multiply result by Constants.kDistancePerEncoderCount
            //^^AKA, multiplying pulses/counts per second by Distance (meters) per pulse/count to get Distance (meters) per second
                                                                        //Convert from 100ms to 1 second
        double leftEncoderVelocity = Constants.kDistancePerEncoderCount*(leftMotor1.getSelectedSensorVelocity()*10); 

        double rightEncoderVelocity = Constants.kDistancePerEncoderCount*(rightMotor1.getSelectedSensorVelocity()*10); 

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
        //double[] initialXandY = {pose.getX(), pose.getY()};
        //SmartDashboard.putNumberArray("Initial Pose", initialXandY);
        resetEncoders();
        odometry.resetPosition(pose, gyro.getRotation2d());
    }

}
