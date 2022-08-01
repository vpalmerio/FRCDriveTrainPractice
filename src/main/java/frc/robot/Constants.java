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

    public static String quickPathTrajectoryJSON = "paths/QuickPath/generatedJSON/QuickPath.wpilib.json";
    public static String circleTrajectoryJSON = "paths/Circle.wpilib.json";
    public static String weirdTrajectoryJSON = "paths/intentionally-weird.wpilib.json";

    public static int xboxControllerPort = 0;

    public static int left1CANPort = 2;
    public static int left2CANPort = 3;
    public static int right1CANPort = 0;
    public static int right2CANPort = 1;

    public static boolean left_side_inverted = true; 
    public static boolean right_side_inverted = !left_side_inverted;


    //currently unrealistic values because we screwed up testing
    public static final double ksVolts = 0.59348;
    public static final double kvVoltSecondsPerMeter = 2.1763; // 2.1763;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15926;
    public static final double kPDriveVel = 2.457;

    //distance between centers of the left side center of one wheel to the right center of the other wheel, across the robot chassis
    public static final double kTrackWidthMeters = 0.6169;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackWidthMeters);
    
    
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    //^^these might have to be tuned

    //tutorial said these values were fine
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    
   
    //Getting distance traveled per encoder cycle
    public static final double kWheelDiameterMeters = 0.14605; //measured

    public static final int kEncoderCPR = 2048; //this is according to Falcon500 documentation
    //encoder counts per revolution (one full turn of motor shaft), also, pulses can be interpreted as counts

        //factor in gear ratio to determine how far robot travels per count
        //first gear ratio, 9:62
        //second gear ratio, 20:28
        //multiply the two to get 180:1736
        //its a gear reduction (for every turn of the motor shaft, the robot wheel only turns a fraction of its full revolution)
            //^^flip gear ratios 1736:180 (sys id requires it flipped like this, as far as I understand)
            //convert to x/1 -> 1736/180:180/180 = 9.6444... : 1
        //the motor shaft must turn 9.6444.. times for the robot wheel to make a full revolution
    public static final double kGearRatio = 1736/180;

    //multiply 9.6444.. by 2048 to get the amount of encoder counts for the wheel to turn once
    public static final double kEncoderCountsPerFullWheelTurn = kGearRatio*kEncoderCPR;
    //FINALLY, divide the wheel circumference by the result from the last step to get the distance per encoder counts!!
    public static final double kDistancePerEncoderCount = (kWheelDiameterMeters * Math.PI) / kEncoderCountsPerFullWheelTurn;
}
