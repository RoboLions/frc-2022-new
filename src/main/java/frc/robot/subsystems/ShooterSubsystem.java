// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Constants.ShooterConstants;
import frc.robot.lib.RoboLionsPID;

public class ShooterSubsystem extends SubsystemBase {

  public static final double RIGHT_LOW_HUB_SHOOTER_POWER = 0.24;
  public static final double LEFT_LOW_HUB_SHOOTER_POWER = -0.24;
  
  public static final double LEFT_HOPPER_IN_POWER = -0.3;
  public static final double RIGHT_HOPPER_IN_POWER = 0.3;
  public static final double LEFT_HOPPER_OUT_POWER = 0.2;
  public static final double RIGHT_HOPPER_OUT_POWER = -0.2;

  public static final double LEFT_MOVE_BELT_UP_POWER = 0.3;
  public static final double LEFT_MOVE_BELT_DOWN_POWER = -0.3;
  public static final double RIGHT_MOVE_BELT_UP_POWER = -0.3;
  public static final double RIGHT_MOVE_BELT_DOWN_POWER = 0.3;
   
  // Constants
  public final double MAX_DESIRED_RPM = 2000.0;
  public final double MINUTES_TO_MS = 0.001666666666; // equal to 1/600
  public final double ENC_COUNTS_PER_REV = 2048.0;

  // Conversion Factor to turn encoder ticks/100ms into Meters per Second
  public static final double TICKS_PER_METER = (2048 * 12.75 * 10) / (5.0);
  private static final double METERS_PER_TICKS = 1 / TICKS_PER_METER;

  public RoboLionsPID leftShooterPID = new RoboLionsPID();
  public RoboLionsPID rightShooterPID = new RoboLionsPID();

  public double left_shoot_speed_cmd;
  public double right_shoot_speed_cmd;

  private static WPI_VictorSPX leftHopperMotor = RobotMap.leftHopperMotor;
  private static WPI_VictorSPX rightHopperMotor = RobotMap.rightHopperMotor;
  private static WPI_VictorSPX frontElevatorMotor = RobotMap.frontElevatorMotor;
  private static WPI_VictorSPX backElevatorMotor = RobotMap.backElevatorMotor;
  private static WPI_TalonFX leftShooterMotor = RobotMap.leftShooterMotor;
  private static WPI_TalonFX rightShooterMotor = RobotMap.rightShooterMotor;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    resetEncoders();
    
    //rightShooterMotor.set(ControlMode.Follower, leftShooterMotor.getDeviceID());

    leftShooterMotor.setNeutralMode(NeutralMode.Coast);
    rightShooterMotor.setNeutralMode(NeutralMode.Coast);

    backElevatorMotor.setNeutralMode(NeutralMode.Coast);
    frontElevatorMotor.setNeutralMode(NeutralMode.Coast);

    leftHopperMotor.setNeutralMode(NeutralMode.Coast);
    rightHopperMotor.setNeutralMode(NeutralMode.Coast);

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

    leftShooterPID.initialize2(
      0, // Proportional Gain
      0, // Integral Gain
      0.0, // Derivative Gain //0
      0.0, // Cage Limit 0.3 //0
      0.0, // Deadband //0
      12,// MaxOutput Volts 0.25 //100 //12
      false, //enableCage
      false //enableDeadband
    );

    rightShooterPID.initialize2(
      0, // Proportional Gain
      0, // Integral Gain
      0, // Derivative Gain 
      0.0, // Cage Limit //0.3
      0.0, // Deadband //0
      12,// MaxOutput Volts 0.25 //100 //12
      false, //enableCage
      false //enableDeadband
    );
  }

  public void stopShooter() {
    leftShooterMotor.set(0);
    rightShooterMotor.set(0);
  }

  public void setModePercentVoltage() {
    leftShooterMotor.set(ControlMode.PercentOutput, 0);
    rightShooterMotor.set(ControlMode.PercentOutput, 0);
  }
  
  public double getLeftEncoderPosition() {
    return leftShooterMotor.getSelectedSensorPosition();
  }

  public double getRightEncoderPosition() {
    return rightShooterMotor.getSelectedSensorPosition();
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

  public double getAverageEncoderVelocityMPS() {
    double velocityMPS = (getRightEncoderVelocityMetersPerSecond() + getLeftEncoderVelocityMetersPerSecond())
                          * 0.5;
    return (velocityMPS);
  }

  // feedforward calculation
  public double calculateNew(double velocity, double acceleration, double ks, double kv, double ka) {
    return ks * Math.signum(velocity) + kv * velocity + ka * acceleration;
  } 

  public void steadyShoot(double velocity) {
    // actual speed command passed 
    left_shoot_speed_cmd = velocity; 
    right_shoot_speed_cmd = velocity;
    
    // calculate rate feedforward term
    final double leftFeedforward = calculateNew(velocity, 0, 0, 0, 0); 
    final double rightFeedforward = calculateNew(velocity, 0, 0, 0, 0);

    double batteryVoltage = RobotController.getBatteryVoltage(); // getting battery voltage from PDP via the rio

    if (batteryVoltage < 1) {
      batteryVoltage = 1;
    }

    // output to compensate for speed error, the PID block
    double leftOutput = leftShooterPID.execute(velocity, getLeftEncoderVelocityMetersPerSecond());
    double rightOutput = rightShooterPID.execute(velocity, getRightEncoderVelocityMetersPerSecond());
    
    // final voltage command going to falcon or talon (percent voltage, max 12 V)
    double LVoltagePercentCommand = ((leftOutput + leftFeedforward) / batteryVoltage);
    double RVoltagePercentCommand = ((rightOutput + rightFeedforward) / batteryVoltage);

    // should never have value above 1 or -1, always in-between
    if (LVoltagePercentCommand > 1.0) {
      LVoltagePercentCommand = 1.0;
    }
    else if (LVoltagePercentCommand < -1.0) {
      LVoltagePercentCommand = -1.0;
    }

    if (RVoltagePercentCommand > 1.0) {
      RVoltagePercentCommand = 1.0;
    }
    else if (RVoltagePercentCommand < -1.0) {
      RVoltagePercentCommand = -1.0;
    }

    SmartDashboard.putNumber("Right Motor Command", RVoltagePercentCommand);
    SmartDashboard.putNumber("Left Motor Command", LVoltagePercentCommand);

    // motors need to move in opposite directions
    if (LVoltagePercentCommand > 0) {
      if (RVoltagePercentCommand > 0) {
        RVoltagePercentCommand = RVoltagePercentCommand * -1;
      }
    }
    else if (LVoltagePercentCommand < 0) {
      if (RVoltagePercentCommand < 0) {
        RVoltagePercentCommand = RVoltagePercentCommand * -1;
      }
    }
    
    leftShooterMotor.set(LVoltagePercentCommand);
    //rightShooterMotor.set(RVoltagePercentCommand);
    
    //SmartDashboard.putNumber("Left Encoder Counts", getLeftEncoderPosition());
    //SmartDashboard.putNumber("Right Encoder Counts", getRightEncoderPosition());
    
    /*
    SmartDashboard.putNumber("Left Dist Meters", leftDistanceTravelledInMeters());
    SmartDashboard.putNumber("Right Dist Meters", rightDistanceTravelledInMeters());*/

    SmartDashboard.putNumber("Left Encoder MPS", getLeftEncoderVelocityMetersPerSecond());
    SmartDashboard.putNumber("Right Encoder MPS", getRightEncoderVelocityMetersPerSecond());
    
    //System.out.println("Left Error:" + (leftSpeed - getBackLeftEncoderVelocityMetersPerSecond()));
    //System.out.println(getLeftEncoderVelocityMetersPerSecond() + "," + getRightEncoderVelocityMetersPerSecond());
    // System.out.println("Left Error: " + (leftSpeed-getLeftEncoderVelocityMetersPerSecond()) + "/ Right Error: " + (getRightEncoderVelocityMetersPerSecond()-rightSpeed));
    // System.out.println("Debug Out  " + rightOutput + " /// " + rightFeedforward + " /// " + JoystickDrive.throttle);
  }

  public double getDistance() {
    // https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
    // TODO: physically measure the distance

    /*double distance = (ShooterConstants.heightOfUpperHubMeters - ShooterConstants.heightOfLimelightMeters) 
    / Math.tan(ShooterConstants.angleOfLimelight + LimelightSubsystem.getLimelightY());*/

    /*
    double distance = 2.286;
*/
    return 0; // horizontal distance in meters
    
  }

  public double getVelocityOfWheel() {
    // big thanks to Crawford's physics brain, finding velocity of ball
    double initialVelocityMPS = 
    Math.sqrt( 
      (4.9 * Math.pow(getDistance(), 2)) / 
      (
        (
          (Math.tan(ShooterConstants.angleOfShooter) * getDistance()) 
          - 
          (ShooterConstants.heightOfUpperHubMeters - ShooterConstants.heightOfShooterMeters)
        )
      * Math.pow(Math.cos(ShooterConstants.angleOfShooter), 2)
      )
    );

    //double initialVelocityMP100MS = (initialVelocityMPS / 10); // divide by 10 to get to 100 MS unit

    // convert to angular speed
    double velocityOfWheel = (initialVelocityMPS / ShooterConstants.radiusOfWheel);
    return (velocityOfWheel / 0.1); // divide by 10 to get 100 MS unit
  }

  public void shootUpperHub() {
    steadyShoot(0.5);
  }

  public void shootLowerHub() {
    leftShooterMotor.set(LEFT_LOW_HUB_SHOOTER_POWER);
    rightShooterMotor.set(RIGHT_LOW_HUB_SHOOTER_POWER);
  }

  public void resetEncoders() {
    leftShooterMotor.setSelectedSensorPosition(0);
    rightShooterMotor.setSelectedSensorPosition(0);
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