// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.lib.RoboLionsPID;

public class ArmSubsystem extends SubsystemBase {

  //0.5 speed to go down
  // 0.7 speed to go up

  // arm = -22 degrees home position
  // 78 = 8 inches

  private final WPI_VictorSPX armMotor = RobotMap.intakeArmMotor;
  public RoboLionsPID armPID = new RoboLionsPID();
  private final Pigeon2 imu = RobotMap.intakeIMU;
  public static double MAX_ARM_POWER = 0.9;
  public static double arm_cmd;

  public double arm_pitch_readout = 0;

  public ArmSubsystem() {
      armMotor.setNeutralMode(NeutralMode.Brake);
      armPID.initialize2( 0.09, // Proportional Gain 0.02
                          0.1, // Integral Gain .311
                          0.0, // Derivative Gain
                          10, // Cage Limit
                          1, // Deadband
                          MAX_ARM_POWER, // MaxOutput hard deadband as to what the maximum possible command is
                          true,
                          true
      );
  }

  public void moveArmToPosition(double target_pitch) {
      arm_pitch_readout = getPitch();
      
      double arm_cmd = armPID.execute((double)target_pitch, (double)arm_pitch_readout);
      // add hard deadband to arm so we don't break it
      if(arm_cmd > MAX_ARM_POWER) {
          arm_cmd = MAX_ARM_POWER;
      } else if(arm_cmd < -MAX_ARM_POWER) {
          arm_cmd = -MAX_ARM_POWER;
      }
      
      System.out.println(target_pitch + ", " + arm_pitch_readout + ", " + arm_cmd);
      //System.out.println("Arm Cmd" + arm_cmd);
      armMotor.set(arm_cmd); // need to invert command to close the loop

      /*
      if (arm_pitch_readout > ArmConstants.HOME_POSITION) {
          armMotor.set(0.6);
      } else if (arm_pitch_readout < ArmConstants.GROUND_POSITION) {
          armMotor.set(-0.6);
      } else {
          armMotor.set(0);
      }*/
  }

  public void setArmToHome() {
      moveArmToPosition(ArmConstants.HOME_POSITION);
  }

  public void setArmToGround() {
      moveArmToPosition(ArmConstants.GROUND_POSITION);
  }

  public void setArmPower(double power) {
      // add hard deadband to arm so we don't break it
      if(power > MAX_ARM_POWER) {
          power = MAX_ARM_POWER;
      } else if(power < -MAX_ARM_POWER) {
          power = -MAX_ARM_POWER;
      }

      if((power > 0 && power < 0.25) || (power < 0 && power > -0.25)) {
          power = 0;
      }

      armMotor.set(power);
  }

  public void stop() {
      armMotor.set(0);
  }

  public double getPitch() {
    /*double[] ypr = new double[3];
    imu.getYawPitchRoll(ypr);
    return ypr[1];*/
    double pitch = imu.getPitch();
    return pitch;
  }

  /*
  public void resetPitch() {
      // TODO Figure out how to set the pitch
      imu.setYaw(0, ArmConstants.TIMEOUT_MS);
  }*/
}