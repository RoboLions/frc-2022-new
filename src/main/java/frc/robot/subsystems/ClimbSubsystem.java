// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.lib.RoboLionsPID;

public class ClimbSubsystem extends SubsystemBase {

  private static WPI_TalonFX rightClimbMotor = RobotMap.rightClimbMotor;
  private static WPI_TalonFX leftClimbMotor = RobotMap.leftClimbMotor;
  private static WPI_TalonFX highRightClimbMotor = RobotMap.highRightClimbMotor;
  private static WPI_TalonFX highLeftClimbMotor = RobotMap.highLeftClimbMotor;
  private static Servo leftServo = RobotMap.servo1;
  private static Servo rightServo = RobotMap.servo2;
  public double climb_enc_readout = 0;
  
  //public RoboLionsPID climbPID = new RoboLionsPID();

  public ClimbSubsystem() {
    //leftServo.setBounds(2, 1.8, 1.5, 1.2, 1);
    //rightServo.setBounds(2, 1.8, 1.5, 1.2, 1);
    rightClimbMotor.setNeutralMode(NeutralMode.Brake);
    leftClimbMotor.setNeutralMode(NeutralMode.Brake);
    highLeftClimbMotor.setNeutralMode((NeutralMode.Brake));
    highRightClimbMotor.setNeutralMode((NeutralMode.Brake));
    rightClimbMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    leftClimbMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    highRightClimbMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    highLeftClimbMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    //resetEncoder();

    /*
    climbPID.initialize(
        0.0, // Proportional Gain
        0.0, // Integral Gain 
        0.0, // Derivative Gain
        20, // Cage Limit
        2, // Deadband
        0.75 // MaxOutput
    );*/
  }

  /*
  public void moveClimbToPosition(double target_position) {
        climb_enc_readout = getEncoderPosition();
        double arm_cmd = climbPID.execute((double)target_position, (double)climb_enc_readout);
        climbMotor.set(arm_cmd); // need to invert command to close the loop
    }

    public void setClimbToColorWheel() {
        moveClimbToPosition(ClimbConstants.COLOR_WHEEL_POSITION);
    }

    public void setClimbToMax() {
        moveClimbToPosition(ClimbConstants.MAX_POSITION);
    }

    public void setClimbToReady() {
        moveClimbToPosition(ClimbConstants.READY_POSITION);
    }

    public void setClimbToGround() {
        moveClimbToPosition(ClimbConstants.DOWN_POSITION);
    }
  }*/

  public void resetEncoder() {
    rightClimbMotor.setSelectedSensorPosition(0);
    leftClimbMotor.setSelectedSensorPosition(0);
  }

  // move servos out to climb
  public void moveServoOut() {
    leftServo.set(1);
    rightServo.set(0);
  }

  // keep servos in instead of neutral power
  public void moveServosIn() {
    leftServo.set(0);
    rightServo.set(1);
  }

  public double getRightEncoderPosition() {
    return rightClimbMotor.getSelectedSensorPosition();
  }

  public double getLeftEncoderPosition() {
    return leftClimbMotor.getSelectedSensorPosition();
  }

  public void setLeftClimbPower(double power) {
    leftClimbMotor.set(power);
  }

  public void setRightClimbPower(double power) {
    rightClimbMotor.set(power);
  }

  public void setRHighClimbPower(double power) {
    highRightClimbMotor.set(power);
  }

  public void setLHighClimbPower(double power) {
    highLeftClimbMotor.set(power);
  }

  public void stopClimb() {
    rightClimbMotor.set(0.0);
    leftClimbMotor.set(0.0);
  }

  public int getRightLimitSwitchValue() {
    return rightClimbMotor.getSensorCollection().isFwdLimitSwitchClosed(); // 1 if closed, 0 if open
  }

  public int getLeftLimitSwitchValue() {
    return leftClimbMotor.getSensorCollection().isRevLimitSwitchClosed(); // 1 if closed, 0 if open
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}