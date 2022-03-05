// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
//import frc.robot.Constants.ShooterConstants;

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

  public static final double TICKS_PER_METER = (2048 * 12.75 * 10) / (5.0);
  private static final double METERS_PER_TICKS = 1 / TICKS_PER_METER;

  //private static WPI_VictorSPX leftHopperMotor = RobotMap.leftHopperMotor;
  //private static WPI_VictorSPX rightHopperMotor = RobotMap.rightHopperMotor;
  private static WPI_VictorSPX frontElevatorMotor = RobotMap.frontElevatorMotor;
  private static WPI_VictorSPX backElevatorMotor = RobotMap.backElevatorMotor;
  private static WPI_TalonFX leftShooterMotor = RobotMap.leftShooterMotor;
  private static WPI_TalonFX rightShooterMotor = RobotMap.rightShooterMotor;
  
  /** Creates a new SimpleShooterSubsystem. */
  public SimpleShooterSubsystem() {
    RobotMap.leftShooterMotor.setNeutralMode(NeutralMode.Coast);
    RobotMap.rightShooterMotor.setNeutralMode(NeutralMode.Coast);
    backElevatorMotor.setNeutralMode(NeutralMode.Coast);
    frontElevatorMotor.setNeutralMode(NeutralMode.Coast);
    //leftHopperMotor.setNeutralMode(NeutralMode.Coast);
    //rightHopperMotor.setNeutralMode(NeutralMode.Coast);
  }

  public void shootLowerHub() {
    RobotMap.leftShooterMotor.set(LEFT_LOW_HUB_SHOOTER_POWER);
    RobotMap.rightShooterMotor.set(RIGHT_LOW_HUB_SHOOTER_POWER);
  }

  public void shootUpperHub() {
    /*RobotMap.leftShooterMotor.set(LEFT_UPPER_HUB_SHOOTER_POWER);
    RobotMap.rightShooterMotor.set(RIGHT_UPPER_HUB_SHOOTER_POWER);*/
    RobotMap.leftShooterMotor.set(0.2);
    RobotMap.rightShooterMotor.set(-0.2);
  }

  public void stopShooter() {
    RobotMap.leftShooterMotor.set(0);
    RobotMap.rightShooterMotor.set(0);
  }

  /*
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
	}*/

  public double getLeftEncoderVelocity() {
    return leftShooterMotor.getSelectedSensorVelocity();
  }

  public double getRightEncoderVelocity() {
    return rightShooterMotor.getSelectedSensorVelocity();
  }

  public double getLeftEncoderVelocityMetersPerSecond() {
    // getQuadVelocity is in 100 ms so we have to divide it by 10 to get seconds
    double leftVelocityMPS = (leftShooterMotor.getSelectedSensorVelocity() * 10); // /10
    // since getQuadVelocity is in encoder ticks, we have to convert it to meters
    leftVelocityMPS = leftVelocityMPS * METERS_PER_TICKS;
    return (leftVelocityMPS);
  }

  public double getRightEncoderVelocityMetersPerSecond() {
    // getQuadVelocity is in 100 ms so we have to divide it by 10 to get seconds
    double rightVelocityMPS = (rightShooterMotor.getSelectedSensorVelocity() * 10); // /10
    // since getQuadVelocity is in encoder ticks, we have to convert it to meters
    // Need to have a negative for right velocity since the motors are reversed on
    // the opposite side
    rightVelocityMPS = rightVelocityMPS * METERS_PER_TICKS;
    return (rightVelocityMPS);
  }

  public double getAverageEncoderVelocityMPS() {
    double velocityMPS = (getRightEncoderVelocityMetersPerSecond() + getLeftEncoderVelocityMetersPerSecond())
                          * 0.5;
    return (velocityMPS);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println("Encoder Velocity: " + getAverageEncoderVelocityMPS());
  }
}
