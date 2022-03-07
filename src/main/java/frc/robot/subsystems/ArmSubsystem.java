// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;
//import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.lib.RoboLionsPID;

public class ArmSubsystem extends SubsystemBase {

    private final WPI_VictorSPX intakeArmMotor = RobotMap.intakeArmMotor;
    public RoboLionsPID armPID = new RoboLionsPID();
    private final Pigeon2 intakeIMU = RobotMap.intakeIMU;
    //public static double MAX_ARM_POWER = 0.5; //0.3; // hard deadband as to what the maximum possible command is

    public double arm_pitch_readout = 0;

    public ArmSubsystem() {
        intakeArmMotor.setNeutralMode(NeutralMode.Brake);
        /*armPID.initialize2(ArmConstants.kP, // Proportional Gain
                            ArmConstants.kI, // Integral Gain
                            ArmConstants.kD, // Derivative Gain
                            5, // Cage Limit
                            1, // Deadband
                            MAX_ARM_POWER, // MaxOutput
                            false,
                            true
        );*/
    }

    /*
    public void moveArmToPosition(double target_pitch) {
        arm_pitch_readout = getPitch();
        //double arm_cmd = armPID.execute((double)target_pitch, (double)arm_pitch_readout);
        // add hard deadband to arm so we don't break it
        /*if(arm_cmd > MAX_ARM_POWER) {
            arm_cmd = MAX_ARM_POWER;
        } else if(arm_cmd < -MAX_ARM_POWER) {
            arm_cmd = -MAX_ARM_POWER;
        }
        intakeArmMotor.set(arm_cmd); // need to invert command to close the loop
    }

    public void setArmToGround() {
      moveArmToPosition(75);
    }

    public void setArmToStart() {
      moveArmToPosition(-21);
    }*/

    public void setArmPower(double power) {
      intakeArmMotor.set(power);
    }

    public void stop() {
      intakeArmMotor.set(0);
    }

    public double getPitch() {
    	double[] ypr = new double[3];
    	intakeIMU.getYawPitchRoll(ypr);
    	return ypr[1];
    }

    /*
    public void resetPitch() {
      intakeIMU.setYaw(0, 10);
    }*/
  }