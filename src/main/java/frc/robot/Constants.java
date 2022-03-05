// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int MANIPULATOR_CONTROLLER_PORT = 1;
    }

    /*
    public static final class ShooterConstants {
        //TODO: find the right values
        public static final double angleOfLimelight = 33; // degrees, mounting angle of LL

        public static final double heightOfUpperHubMeters = 2.6416; // meters
        public static final double heightOfUpperHubFeet = 8.6666; // feet

        public static final double heightOfLimelightFeet = 3.5; // feet from top of LL to ground
        public static final double heightOfLimelightMeters = 1.0668; // meters from top of LL to ground

        public static final double angleOfShooter = 70; // degrees, angle ball is shooting out
        public static final double heightOfShooterMeters = 1.09728; // meeters from ground to shooter

        public static final double radiusOfWheel = 0.0508; // 2 inches to meters

        public static final double kShooterFreeRPS = 106.33;
        public static final double kShooterToleranceRPS = 50;

    }*/

    public static final class LimelightConstants {
        //modes for limelight led light
        public static final double FORCE_OFF = 1;
        public static final double FORCE_BLINK = 2;
        public static final double FORCE_ON = 3;
    
        //modes for limelight camera 
        public static final double VISION_PROCESSOR = 0;
        public static final double DRIVER_CAMERA = 1;
        public static final double LLAIMING = 0.035;
        public static final double MOTORGAIN = 0.75; //0.8  //0.7; //0.6
    }

    /*public static final class LIDARConstants {
        public static final double offset  = 0; //offset for sensor. test with tape measure
    }*/
}
