// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
//import frc.robot.Constants.ShooterConstants;
import frc.robot.lib.RoboLionsPID;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class ShooterSubsystem extends SubsystemBase {

  private static WPI_VictorSPX frontElevatorMotor = RobotMap.frontElevatorMotor;
  private static WPI_VictorSPX backElevatorMotor = RobotMap.backElevatorMotor;
  private static WPI_TalonFX leftShooterMotor = RobotMap.leftShooterMotor;
  private static WPI_TalonFX rightShooterMotor = RobotMap.rightShooterMotor;
  private static WPI_TalonFX hoodMotor = RobotMap.shooterHoodMotor;

  public static final double LEFT_MOVE_BELT_UP_POWER = 0.5;
  public static final double LEFT_MOVE_BELT_DOWN_POWER = -0.3;
  public static final double RIGHT_MOVE_BELT_UP_POWER = -0.5;
  public static final double RIGHT_MOVE_BELT_DOWN_POWER = 0.3;

  // Conversion Factor to turn encoder ticks/100ms into Meters per Second
  public static final double TICKS_PER_METER = (2048 * 12.75 * 10) / (5.0);
  private static final double METERS_PER_TICKS = 1 / TICKS_PER_METER;

  private static final int MOTOR_ENCODER_COUNTS_PER_REV = 2048;
  
  public RoboLionsPID shooterPID = new RoboLionsPID();

  public static double lastShootVelocity = 0;
  
  public double shoot_speed_cmd;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    leftShooterMotor.setNeutralMode(NeutralMode.Coast);
    rightShooterMotor.setNeutralMode(NeutralMode.Coast);
    hoodMotor.setNeutralMode(NeutralMode.Coast);

    backElevatorMotor.setNeutralMode(NeutralMode.Coast);
    frontElevatorMotor.setNeutralMode(NeutralMode.Coast);

    leftShooterMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    rightShooterMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
  
    leftShooterMotor.configNominalOutputForward(0, 10);
    rightShooterMotor.configNominalOutputReverse(0, 10);

    leftShooterMotor.configNeutralDeadband(0.001, 10);
    rightShooterMotor.configNeutralDeadband(0.001, 10);

    rightShooterMotor.configNominalOutputForward(0, 10);
    rightShooterMotor.configNominalOutputReverse(0, 10);
    rightShooterMotor.configPeakOutputForward(1, 10);
    rightShooterMotor.configPeakOutputReverse(-1, 10);

    leftShooterMotor.configNominalOutputForward(0, 10);
    leftShooterMotor.configNominalOutputReverse(0, 10);
    leftShooterMotor.configPeakOutputForward(1, 10);
    leftShooterMotor.configPeakOutputReverse(-1, 10);
    leftShooterMotor.configNeutralDeadband(0.001, 10);

    leftShooterMotor.configAllowableClosedloopError(0, 0, 10);
    rightShooterMotor.configAllowableClosedloopError(0, 0, 10);

    shooterPID.initialize2(
      2.15, // Proportional Gain 3.15, 2.15
      7, // Integral Gain 12.19, 7
      0, // Derivative Gain //0
      3, // 25% of peak 12V voltage, Cage Limit
      0.0, // Deadband //0
      3, // 25% of peak 12V voltage, MaxOutput Volts
      true, //enableCage make sure the integrator does not charge internally when the output climbs past 3.0 volts
      false //enableDeadband
    );
  }

  // feedforward calculation
  public double calculateNew(double velocity, double acceleration, double ks, double kv, double ka) {
    return ks * Math.signum(velocity) + kv * velocity + ka * acceleration;
  } 

  public void setSpeed(double x){
    leftShooterMotor.set(-x);
    rightShooterMotor.set(x);
  }

  public void steadyShoot(double velocity) {

    // Steps:
    // 1 - decide accel or decel rn
    // 2 - limit commanded velocity based on computed accel limit

    /*
    double linearAccel = (velocity - lastShootVelocity)/0.6;
    double accelLimit = 2; //meters per second

    // are we accel or decel? part 1
    if (lastShootVelocity > 0) {
      if (linearAccel < 0) {
        // velocity pos, accel negative, speed dec (slowing down), use decel
        accelLimit = 4; // meters per second decel
      } else {
        accelLimit = 2; // accel
      }
    } else { // we have negative velocity command
      if (linearAccel > 0) {
        // velocity neg, accel pos, speed dec, decel
        accelLimit = 4; //decel
      } else {
        accelLimit = 2; //accel
      }
    }

    // part 2: limit velocity based on accelLimit
    if (linearAccel > accelLimit) {
      velocity = lastShootVelocity + accelLimit*0.02;
    }
    else if (linearAccel < -accelLimit) {
      velocity = lastShootVelocity - accelLimit*0.02;
    }

    lastShootVelocity = velocity;*/
    //System.out.println("Shooter speed: " + velocity);

    // actual speed command passed
    shoot_speed_cmd = velocity;
    
    // calculate rate feedforward term
    final double shootFeedforward = calculateNew(velocity, 0, 0.8, 2.5, 0); //0.68 2.5

    double batteryVoltage = RobotController.getBatteryVoltage(); // getting battery voltage from PDP via the rio

    if (batteryVoltage < 1) {
      batteryVoltage = 1;
    }

    double shootOutputPID;

    // current velocity
    double velFeedback = getShooterEncoderVelocity();

    // current error between target and current
    double error1 = velocity - velFeedback;

    //
    if (Math.abs(error1) < .5) {
      // output to compensate for speed error, the PID block
      shootOutputPID = shooterPID.execute(velocity, getShooterEncoderVelocity());
    } else {
      shootOutputPID = 0;
    }
   
    //System.out.println("error" + error1 + ", " + "vel feedback: " + velFeedback);
    
    // final voltage command going to falcon or talon (percent voltage, max 12 V)
    shoot_speed_cmd = ((shootOutputPID + shootFeedforward) / batteryVoltage);

    // should never have value above 1 or -1, always in-between
    if (shoot_speed_cmd > 1.0) {
      shoot_speed_cmd = 1.0;
    }
    else if (shoot_speed_cmd < -1.0) {
      shoot_speed_cmd = -1.0;
    }

    //System.out.println(shoot_speed_cmd);
    
    leftShooterMotor.set(-shoot_speed_cmd);
    rightShooterMotor.set(shoot_speed_cmd);
  }

  public boolean isShooterRampedUp() {
    if (Math.abs(shoot_speed_cmd - getShooterEncoderVelocity()) < 0.1) {
      return true;
    } else {
      return false;
    }
  }

  public void stopShooter() {
    leftShooterMotor.set(0);
    rightShooterMotor.set(0);
    hoodMotor.set(0);
  }

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

  public double getShooterEncoderVelocity() {
    double shooterEncoderVelocity = ((getLeftEncoderVelocityMetersPerSecond() * -1) + getRightEncoderVelocityMetersPerSecond())/2;
    return shooterEncoderVelocity;
  }

  public double getAverageEncoderVelocityMPS() {
    double velocityMPS = (getRightEncoderVelocityMetersPerSecond() + getLeftEncoderVelocityMetersPerSecond())
                          * 0.5;
    return (velocityMPS);
  }

  public double getAverageEncoderVelocity100MS() {
    double velocity100MS = (getLeftEncoderVelocity() + getRightEncoderVelocity()) / 2;
    return velocity100MS;
  }

  public double getRPMOfLeftFalcon() {
    double RPMOfLFalcon = getLeftEncoderVelocity() / 3.413;
    return RPMOfLFalcon;
  }

  public double getRPMOfRightFalcon() {
    double RPMOfRFalcon = getRightEncoderVelocity() / 3.413;
    return RPMOfRFalcon;
  }

  public double getRPMOfLeftShooterWheels() {
    double RPMOfLShooterWheels = getRPMOfLeftFalcon() * 1.33;
    return RPMOfLShooterWheels;
  }

  public double getRPMOfRightShooterWheels() {
    double RPMOfRShooterWheels = getRPMOfRightFalcon() * 1.33;
    return RPMOfRShooterWheels;
  }

  public double getVelocityOfFElevator() {
    return frontElevatorMotor.getSelectedSensorVelocity();
  }

  public double getVelocityOfBElevator() {
    return backElevatorMotor.getSelectedSensorVelocity();
  }
  
  public void moveBeltUp() {
    frontElevatorMotor.set(LEFT_MOVE_BELT_UP_POWER);
    backElevatorMotor.set(RIGHT_MOVE_BELT_UP_POWER);
  }
  
  public void stopBelt() {
		frontElevatorMotor.set(0);
    backElevatorMotor.set(0);
	}
  
  public void moveBeltDown() {
    frontElevatorMotor.set(LEFT_MOVE_BELT_DOWN_POWER);
    backElevatorMotor.set(RIGHT_MOVE_BELT_DOWN_POWER);
  }

  public void setHoodSpeed(double hoodSpeed) {
    hoodMotor.set(hoodSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println("Encoder Velocity: " + getAverageEncoderVelocityMPS());
  }
}
