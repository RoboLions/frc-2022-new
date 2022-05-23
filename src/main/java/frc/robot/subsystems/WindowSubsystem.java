// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class WindowSubsystem extends SubsystemBase {
  public static final double IN_POWER =  1.00; // -0.75;
  public static final double OUT_POWER = -1.00; // 0.75

  private static final WPI_VictorSPX windowMotor = RobotMap.windowHoodMotor;
  /** Creates a new WindowSubsystem. */
  public WindowSubsystem() {
    windowMotor.setNeutralMode(NeutralMode.Coast);
    //intakeMotor.set(0);
  }

  public void windowIn() {
    windowMotor.set(IN_POWER);
  }

  public void windowOut() {
    windowMotor.set(OUT_POWER);
  }

  public void stop() {
    windowMotor.set(0);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
