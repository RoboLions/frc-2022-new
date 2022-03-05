// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class LIDARLiteSubsystem extends SubsystemBase {

  private final Counter m_counter;

  public LIDARLiteSubsystem() {
    m_counter = new Counter(new DigitalInput(RobotMap.DIGITAL_INPUT_LIDAR_LITE_PORT));
    m_counter.setMaxPeriod(1.0);
    m_counter.setSemiPeriodMode(true);
    m_counter.reset();
  }

  public double getDistance() {
    double cm;
    while (m_counter.get() < 1) {
      System.out.println("Lidar: waiting for distance measurement");
    }
    /*
     * getPeriod returns time in seconds. The hardware resolution is microseconds.
     * The LIDAR-Lite unit sends a high signal for 10 microseconds per cm of
     * distance.
     */
    /*cm = (m_counter.getPeriod() * 1000000.0 / 10.0) - 18;
    return cm;
  }

  @Override
  public void periodic() {
  }
}*/