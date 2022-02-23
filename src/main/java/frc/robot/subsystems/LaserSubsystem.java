// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class LaserSubsystem extends SubsystemBase {
  /** Creates a new LaserSubsystem. */

  // TODO: make values right
  // this is the proportion for how many inches there are to each volt
  public static double INCHES_PER_VOLT_DIRECT_CURRENT = 0; 
  
  // this is the distance at which the laser stops sensing
  public static double CONSTANT_OF_ZERO_VOLTS = 0; 

  public LaserSubsystem() {}
  
  public double getLaserDistance() {
  double volts = RobotMap.laserVision.getAverageVoltage(); // voltage data from the laser
    double distanceOfLaser = (volts * INCHES_PER_VOLT_DIRECT_CURRENT) + CONSTANT_OF_ZERO_VOLTS;
    return (distanceOfLaser);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
