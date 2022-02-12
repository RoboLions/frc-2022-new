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

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  climbMotor.setNeutralMode(NeutralMode.Brake);

  private static WPI_TalonSRX climbMotor = RobotMap.climbMotor;
  public double climb_enc_readout = 0;

  public ClimbSubsystem() {

  climbMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
  resetEncoder();
  }

  public void resetEncoder() {
    climbMotor.setSelectedSensorPosition(0);
  }

  public void moveClimbToPosition(double target_position) {
        climb_enc_readout = getEncoderPosition();
        double arm_cmd = climbPID.execute((double)target_position, (double)climb_enc_readout);
        climbMotor.set(arm_cmd); // need to invert command to close the loop
    }

  public double getEncoderPosition() {
    return climbMotor.getSelectedSensorPosition();
  }

  public void setClimbtoMax() {
    moveClimbToPosition(ClimbConstants.MAX_POSITION);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
