// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class IntakeSubsystem extends SubsystemBase {

  // TODO tune value to proper
  public static final double IN_POWER = 0.6; 
  public static final double OUT_POWER = -0.6;

  private static final WPI_VictorSPX intakeMotor = RobotMap.intakeMotor;

  public IntakeSubsystem() {
    intakeMotor.setNeutralMode(NeutralMode.Coast);
    //intakeMotor.set(0);
  }

  public void intakeBalls() {
    intakeMotor.set(IN_POWER);
  }

  public void outtakeBalls() {
    intakeMotor.set(OUT_POWER);
  }


  public void stop() {
    intakeMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}