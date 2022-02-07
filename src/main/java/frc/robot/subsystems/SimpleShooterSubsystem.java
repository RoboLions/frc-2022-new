// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Constants.ShooterConstants;

public class SimpleShooterSubsystem extends SubsystemBase {

  public static final double RIGHT_LOW_HUB_SHOOTER_POWER = 0.24;
  public static final double LEFT_LOW_HUB_SHOOTER_POWER = -0.24;
  public static final double RIGHT_UPPER_HUB_SHOOTER_POWER = 0.7;
  public static final double LEFT_UPPER_HUB_SHOOTER_POWER = -0.7;
  
  public static final double LEFT_HOPPER_IN_POWER = -0.3;
  public static final double RIGHT_HOPPER_IN_POWER = 0.3;
  public static final double LEFT_HOPPER_OUT_POWER = 0.2;
  public static final double RIGHT_HOPPER_OUT_POWER = -0.2;

  public static final double LEFT_MOVE_BELT_UP_POWER = 0.3;
  public static final double LEFT_MOVE_BELT_DOWN_POWER = -0.3;
  public static final double RIGHT_MOVE_BELT_UP_POWER = -0.3;
  public static final double RIGHT_MOVE_BELT_DOWN_POWER = 0.3;

  private static WPI_VictorSPX leftHopperMotor = RobotMap.leftHopperMotor;
  private static WPI_VictorSPX rightHopperMotor = RobotMap.rightHopperMotor;
  private static WPI_VictorSPX frontElevatorMotor = RobotMap.frontElevatorMotor;
  private static WPI_VictorSPX backElevatorMotor = RobotMap.backElevatorMotor;
  
  /** Creates a new SimpleShooterSubsystem. */
  public SimpleShooterSubsystem() {
    RobotMap.leftShooterMotor.setNeutralMode(NeutralMode.Coast);
    RobotMap.rightShooterMotor.setNeutralMode(NeutralMode.Coast);
    backElevatorMotor.setNeutralMode(NeutralMode.Coast);
    frontElevatorMotor.setNeutralMode(NeutralMode.Coast);
    leftHopperMotor.setNeutralMode(NeutralMode.Coast);
    rightHopperMotor.setNeutralMode(NeutralMode.Coast);
  }

  public void shootLowerHub() {
    RobotMap.leftShooterMotor.set(LEFT_LOW_HUB_SHOOTER_POWER);
    RobotMap.rightShooterMotor.set(RIGHT_LOW_HUB_SHOOTER_POWER);
  }

  public void shootUpperHub() {
    RobotMap.leftShooterMotor.set(LEFT_UPPER_HUB_SHOOTER_POWER);
    RobotMap.rightShooterMotor.set(RIGHT_UPPER_HUB_SHOOTER_POWER);
  }

  public void stopShooter() {
    RobotMap.leftShooterMotor.set(0);
    RobotMap.rightShooterMotor.set(0);
  }

  public void moveBeltUp() {
    leftHopperMotor.set(LEFT_HOPPER_IN_POWER);
    rightHopperMotor.set(RIGHT_HOPPER_IN_POWER);
    frontElevatorMotor.set(LEFT_MOVE_BELT_UP_POWER);
    backElevatorMotor.set(RIGHT_MOVE_BELT_UP_POWER);
  }

  public void moveBeltDown() {
    leftHopperMotor.set(LEFT_HOPPER_OUT_POWER);
    rightHopperMotor.set(RIGHT_HOPPER_OUT_POWER);
    frontElevatorMotor.set(LEFT_MOVE_BELT_DOWN_POWER);
    backElevatorMotor.set(RIGHT_MOVE_BELT_DOWN_POWER);
  }
  
  public void stopBelt() {
		frontElevatorMotor.set(0);
    backElevatorMotor.set(0);
    leftHopperMotor.set(0);
    rightHopperMotor.set(0);
	}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
