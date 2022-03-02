// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;

public class LimelightSubsystem extends SubsystemBase {

  public static NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  public static NetworkTableEntry tx = limelightTable.getEntry("tx");
  public static NetworkTableEntry ty = limelightTable.getEntry("ty");

  public static double limelight_x = (Double) tx.getDouble(0.0);
  public static double limelight_y = (Double) ty.getDouble(0.0);

  public LimelightSubsystem() {
    limelightTable.getEntry("ledMode").setNumber(LimelightConstants.FORCE_ON);   //1 = force off, 2 = force blink, 3 = force on
    limelightTable.getEntry("camMode").setNumber(LimelightConstants.VISION_PROCESSOR);   //0 = Vision processor, 1 = Driver Camera
  }

  /*************************************************************************
  * Call this during a real time periodic thread to constantly refresh data
  *************************************************************************/
  public static void getLimelightData()  {
    limelight_x = (Double) limelightTable.getEntry("tx").getDouble(0.0);
    limelight_y = (Double) limelightTable.getEntry("ty").getDouble(0.0);
  }
  /*************************************************************************
  * Use this to get the limelight x
  *************************************************************************/
  public static double getLimelightX() {
    return (Double) limelightTable.getEntry("tx").getDouble(0.0);
  }
  /*************************************************************************
  * Use this to get the limelight y
  *************************************************************************/
  public static double getLimelightY() {
    return (Double) limelightTable.getEntry("ty").getDouble(0.0);
  }
  /*************************************************************************
  * get the network table object
  *************************************************************************/
  static NetworkTable getTable() {
    return NetworkTableInstance.getDefault().getTable("limelight");
  }
  /*************************************************************************
  * call this function to turn the led ON
  *************************************************************************/
  public static void turn_LED_ON()  {
    limelightTable.getEntry("ledMode").setNumber(LimelightConstants.FORCE_ON);
  }
  /*************************************************************************
  * call this function to turn the led OFF
  *************************************************************************/
  public static void turn_LED_OFF()  {
    limelightTable.getEntry("ledMode").setNumber(LimelightConstants.FORCE_OFF);
  }
  /*************************************************************************
  * call this function to cause the led to BLINK
  *************************************************************************/
  public static void turn_LED_FLASH_BLINK()  {
    limelightTable.getEntry("ledMode").setNumber(LimelightConstants.FORCE_BLINK);
  }
  /*************************************************************************
  * Use this to set the limelight to the regular driver camera mode
  *************************************************************************/
  public static void setDriverCamera() {
    limelightTable.getEntry("camMode").setNumber(LimelightConstants.DRIVER_CAMERA);
    turn_LED_OFF();
  }
  /*************************************************************************
  * Use this to set the limelight to vision processor mode
  *************************************************************************/
  public static void setVisionProcessor() {
    limelightTable.getEntry("camMode").setNumber(LimelightConstants.VISION_PROCESSOR);
    turn_LED_ON();
  }
  /*************************************************************************
  * Allows you to toggle limelight between driver camera and vision
  * processing camera to limit bandwidth use of cameras
  *************************************************************************/
  public void limelightCameraToggle() {
    boolean toggle = false;
    boolean camera = true;
        if (toggle) { // && RobotContainer.getDriverJoystick().getRawButtonPressed(RobotContainer.BUTTON_Y)
        //camera is set to false default and toggle is set to true at default
        toggle = false;  
        // ^ This prevents this section of code from being called again until the Button is released and re-pressed
            if (camera) {  
            camera = false;
            LimelightSubsystem.setDriverCamera();
            System.out.println("DRIVER DRIVER DRIVER DRIVER DRIVER DRIVER");
            } else {
            camera = true;
            LimelightSubsystem.setVisionProcessor();
            System.out.println("VISION VISION VISION VISION VISION VISION");
            }
        } else { //!RobotContainer.getDriverJoystick().getRawButtonPressed(RobotContainer.BUTTON_Y)
            toggle = true; 
        // ^ Button has been released, so this allows a re-press to activate the code above.
        }
  }
  /*************************************************************************
  * Calculate horizontal distance from goal using goal height, limelight
  * height, and limelight mounting angle
  *************************************************************************/
  public static double getHorizontalDistance() {
    double targetOffsetAngle_Vertical = Math.abs(getLimelightY());
    double limelightMountAngleDegrees = 27.5;
    double limelightHeight = 2.916; // feet
    double goalHeight = 8.6666; // feet
    double angleToGoal = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoal * (3.14159 / 180.0);
    double distance = (goalHeight - limelightHeight)/Math.tan(angleToGoalRadians);
    return distance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}