// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.lib.RoboLionsPID;

public class ClimbSubsystem extends SubsystemBase {

  private static WPI_TalonFX rightClimbMotor = RobotMap.rightClimbMotor;
  private static WPI_TalonFX leftClimbMotor = RobotMap.leftClimbMotor;
  public double climb_enc_readout = 0;
  
  //public RoboLionsPID climbPID = new RoboLionsPID();

  public ClimbSubsystem() {

    rightClimbMotor.setNeutralMode(NeutralMode.Brake);
    leftClimbMotor.setNeutralMode(NeutralMode.Brake);
    rightClimbMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    leftClimbMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    //resetEncoder();

/*
    climbPID.initialize2(
        0.0, // Proportional Gain
        0.0, // Integral Gain 
        0.0, // Derivative Gain
        0.0, // Cage Limit 
        0.0, // Deadband 
        12, // MaxOutput Volts
        false, //enableCage
        false //enableDeadband
    );*/
  }

  /*
  public void moveClimbToPosition(double target_position) {
        climb_enc_readout = getEncoderPosition();
        double arm_cmd = climbPID.execute((double)target_position, (double)climb_enc_readout);
        rightClimbMotor.set(arm_cmd); // need to invert command to close the loop
    }

  public void setClimbtoMax() {
    moveClimbToPosition(ClimbConstants.MAX_POSITION);
  }

  public void setClimbMinimum() {
    moveClimbToPosition(ClimbConstants.DOWN_POSITION);
  }*/

  public void resetEncoder() {
    rightClimbMotor.setSelectedSensorPosition(0);
    leftClimbMotor.setSelectedSensorPosition(0);
  }

  public double getRightEncoderPosition() {
    return rightClimbMotor.getSelectedSensorPosition();
  }

  public double getLeftEncoderPosition() {
    return leftClimbMotor.getSelectedSensorPosition();
  }

  public void setClimbPower(double power) {
    rightClimbMotor.set(power);
    leftClimbMotor.set(power);
  }

  public void stopClimb() {
    rightClimbMotor.set(0.0);
    leftClimbMotor.set(0.0);
  }

  public int getRightLimitSwitchValue() {
    return rightClimbMotor.getSensorCollection().isFwdLimitSwitchClosed(); // 1 if closed, 0 if open
  }

  public int getLeftLimitSwitchValue() {
    return leftClimbMotor.getSensorCollection().isFwdLimitSwitchClosed(); // 1 if closed, 0 if open
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}