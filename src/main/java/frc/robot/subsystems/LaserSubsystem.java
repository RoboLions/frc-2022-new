// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.RobotMap;

// public class LaserSubsystem extends SubsystemBase {
//   /** Creates a new LaserSubsystem. */
//   // TODO: these are all placeholder values atm

//   private static final double MIN_DIST = 0;

//   private static final double MAX_DIST = 0;


//   // this is the proportion for how many inches there are to each volt
//   public static double INCHES_PER_VOLT_DIRECT_CURRENT = 0; 
  
//   // this is the distance at which the laser stops sensing
//   public static double CONSTANT_OF_ZERO_VOLTS = 0; 

//   public LaserSubsystem() {}
  
//   public double getLaserDistance() {
//   double volts = RobotMap.laserVision.getAverageVoltage(); // voltage data from the laser
//     double distanceOfLaser = (volts * INCHES_PER_VOLT_DIRECT_CURRENT) + CONSTANT_OF_ZERO_VOLTS;
//     return (distanceOfLaser);
//   }
  
//   public boolean isRobotReadyToClimb() {
//     boolean robotReadyToClimb = false;
//     double distance = getLaserDistance();
//     if (distance > MIN_DIST && distance < MAX_DIST) {
//       robotReadyToClimb = false;
//     } else {
//       robotReadyToClimb = true;
//     }
    
//     return robotReadyToClimb;
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }
// }
