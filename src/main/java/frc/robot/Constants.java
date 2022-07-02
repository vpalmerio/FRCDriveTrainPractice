// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static int xboxControllerPort = 1;

    public static int left1CANPort = 2;
    public static int left2CANPort = 3;
    public static int right1CANPort = 0;
    public static int right2CANPort = 1;

    //currently unrealistic values because we screwed up testing
    public static final double ksVolts = 0.52644;
    public static final double kvVoltSecondsPerMeter = 0.023164;
    public static final double kaVoltSecondsSquaredPerMeter = 0.00054074;
    public static final double kPDriveVel = 0.011566;

    //distance between centers of the front and back wheels on one side of the robot chassis
    //I do not have this value on hand so I am using tutorials values
    public static final double kTrackWidthMeters = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackWidthMeters);
    
    
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    //^^these might have to be tuned

    //tutorial said these values were fine
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final int kEncoderCPR = 2048;
    public static final double kWheelDiameterMeters = 0.15;
    //^^needs to be changed
    public static final double kEncoderDistancePerPulse = 
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

}
