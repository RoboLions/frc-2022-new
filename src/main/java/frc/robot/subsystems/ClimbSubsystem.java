// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Constants.ClimbConstants;
import frc.robot.lib.RoboLionsPID;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
 
  private static WPI_TalonSRX climbMotor = RobotMap.climbMotor;
  public double climb_enc_readout = 0;
  
  public RoboLionsPID climbPID = new RoboLionsPID();


  public ClimbSubsystem() {

  climbMotor.setNeutralMode(NeutralMode.Brake);

  climbMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
  resetEncoder();

  climbPID.initialize2(
        0.0, // Proportional Gain
        0.0, // Integral Gain 
        0.0, // Derivative Gain
        0.0, // Cage Limit 
        0.0, // Deadband 
        12, // MaxOutput Volts
        false, //enableCage
        false //enableDeadband
    );
  }

  public void moveClimbToPosition(double target_position) {
        climb_enc_readout = getEncoderPosition();
        double arm_cmd = climbPID.execute((double)target_position, (double)climb_enc_readout);
        climbMotor.set(arm_cmd); // need to invert command to close the loop
    }

  public void setClimbtoMax() {
    moveClimbToPosition(ClimbConstants.MAX_POSITION);
  }

  public void setClimbMinimum() {
    moveClimbToPosition(ClimbConstants.DOWN_POSITION);
  }

  public void resetEncoder() {
    climbMotor.setSelectedSensorPosition(0);
  }

  public double getEncoderPosition() {
    return climbMotor.getSelectedSensorPosition();
  }

  public void setClimbPower(double power) {
    climbMotor.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}