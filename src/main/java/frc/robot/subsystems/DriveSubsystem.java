// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.Constants.OIConstants;
import frc.robot.lib.RoboLionsMotionProfile;
import frc.robot.lib.RoboLionsPID;

public class DriveSubsystem extends SubsystemBase {
  public static final double kMaxSpeed = 3.0; // 3.0; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // 2 * Math.PI; // one rotation per second
  private static final double IN_TO_M = .0254;
  
  private static final int timeoutMs = 10;
  private static final int MOTOR_ENCODER_CODES_PER_REV = 2048; //4096 for CTRE Mag Encoders, 2048 for the Falcons
  private static final double DIAMETER_INCHES = 6.0; // wheels on prototype bot
  
  private static final double WHEEL_DIAMETER = DIAMETER_INCHES * IN_TO_M; // in meters
  private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
  // This is for a output gear side motor encoder
  // private static final double TICKS_PER_METER = MOTOR_ENCODER_CODES_PER_REV / WHEEL_CIRCUMFERENCE;

  private static final double GEAR_RATIO = 10.71; // ratio for prototype bot

  // This is for an encoder mounted to the motor
  private static final double TICKS_PER_METER = (MOTOR_ENCODER_CODES_PER_REV * GEAR_RATIO) / (WHEEL_CIRCUMFERENCE);
  private static final double METERS_PER_TICKS = 1 / TICKS_PER_METER;

  // TODO: change - center of wheel to wheel horizontally
  private static final double BOT_WHEEL_TO_WHEEL_DIAMETER = 0.55; // 22 inches to meters

  public boolean state_flag_motion_profile = true;

  //90 degrees /360 = 2*PI*R 
  private static final double HEADING_BOT_DEG_TO_BOT_WHEEL_DISTANCE = (BOT_WHEEL_TO_WHEEL_DIAMETER * Math.PI)/360.0;

  private static final WPI_TalonFX leftBackMotor = RobotMap.leftBackDriveMotor;
  private static final WPI_TalonFX rightBackMotor = RobotMap.rightBackDriveMotor;
  private static final WPI_TalonFX leftFrontMotor = RobotMap.leftFrontDriveMotor;
  private static final WPI_TalonFX rightFrontMotor = RobotMap.rightFrontDriveMotor;

  private static WPI_TalonFX leftShooterMotor = RobotMap.leftShooterMotor;

  private final Pigeon2 imu = RobotMap.chasisIMU;

  // private final static XboxController driverController = RobotContainer.driverController;

  public RoboLionsPID leftForwardPID = new RoboLionsPID();
  public RoboLionsPID rightForwardPID = new RoboLionsPID();
	public RoboLionsPID headingPID = new RoboLionsPID();
  public RoboLionsPID positionPID = new RoboLionsPID();
  public RoboLionsPID limelightRotationPID = new RoboLionsPID();
  public RoboLionsMotionProfile positionMotionProfile = new RoboLionsMotionProfile();
  public RoboLionsMotionProfile headingMotionProfile = new RoboLionsMotionProfile();
  
  public double left_speed_feedback;
  public double right_speed_feedback;

  public double left_speed_cmd;
  public double right_speed_cmd;

  static double lastLinearVelocity = 0;
  static double lastRotateVelocity = 0;
  
  public DriveSubsystem() {
    //ZeroYaw();
    resetEncoders();

    leftFrontMotor.set(ControlMode.Follower, leftBackMotor.getDeviceID());
    rightFrontMotor.set(ControlMode.Follower, rightBackMotor.getDeviceID());

    leftBackMotor.setNeutralMode(NeutralMode.Coast);
    rightBackMotor.setNeutralMode(NeutralMode.Coast);
    // leftFrontMotor.setNeutralMode(NeutralMode.Coast);
    // rightFrontMotor.setNeutralMode(NeutralMode.Coast);

    rightBackMotor.setInverted(false);
    rightFrontMotor.setInverted(false);
    leftFrontMotor.setInverted(true);
    leftBackMotor.setInverted(true);

    leftBackMotor.setSensorPhase(true);
    //rightBackMotor.setSensorPhase(false);

    leftFrontMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    rightFrontMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    leftBackMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    rightBackMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);

    leftFrontMotor.configNominalOutputForward(0, 10);
    leftFrontMotor.configNominalOutputReverse(0, 10);
    leftFrontMotor.configPeakOutputForward(1, 10);
    leftFrontMotor.configPeakOutputReverse(-1, 10);
    leftFrontMotor.configNeutralDeadband(0.001, 10);

    rightFrontMotor.configNominalOutputForward(0, 10);
    rightFrontMotor.configNominalOutputReverse(0, 10);
    rightFrontMotor.configPeakOutputForward(1, 10);
    rightFrontMotor.configPeakOutputReverse(-1, 10);
    rightFrontMotor.configNeutralDeadband(0.001, 10);

    leftBackMotor.configNominalOutputForward(0, 10);
    leftBackMotor.configNominalOutputReverse(0, 10);
    leftBackMotor.configPeakOutputForward(1, 10);
    leftBackMotor.configPeakOutputReverse(-1, 10);
    leftBackMotor.configNeutralDeadband(0.001, 10);

    rightBackMotor.configNominalOutputForward(0, 10);
    rightBackMotor.configNominalOutputReverse(0, 10);
    rightBackMotor.configPeakOutputForward(1, 10);
    rightBackMotor.configPeakOutputReverse(-1, 10);
    rightBackMotor.configNeutralDeadband(0.001, 10);

    // Sets how much error is allowed
    leftFrontMotor.configAllowableClosedloopError(0, 0, 10);
    leftBackMotor.configAllowableClosedloopError(0, 0, 10);
    rightFrontMotor.configAllowableClosedloopError(0, 0, 10);
    rightBackMotor.configAllowableClosedloopError(0, 0, 10);

    // Rate Drive PID
    leftForwardPID.initialize2(
      //0, 0, 0, 0, 0, 12, false, false
      
      // 0.5 = 2.94, 28, 0.077
      3.375, // Proportional Gain 4.9, 7.5, 2.205, 3.375
      //.21 seconds
      17.357, //*0.1, // Integral Gain //12.6 42.12 ZN w FF, 17.357
      0.0, // Derivative Gain //0
      0.0, // Cage Limit 0.3 //0
      0.0, // Deadband //0
      12,// MaxOutput Volts 0.25 //100 //12
      false, //enableCage
      false //enableDeadband
    );

    // Rate Drive PID
    rightForwardPID.initialize2(
      //0, 0, 0, 0, 0, 12, false, false
      
      3.15, // 2.025 Proportional Gain 4.5, 7 //2.925 ZN w FF //2
      16.2, //*0.1, // 10.2316 Integral Gain //42.12 ZN w FF //20
      0, // Derivative Gain //0
      0.0, // Cage Limit //0.3
      0.0, // Deadband //0
      12,// MaxOutput Volts 0.25 //100 //12
      false, //enableCage
      false //enableDeadband
    );

    // Position Command PID for Autonomous and 
    positionPID.initialize2(
      5, // Proportional Gain //2.15 //1.35 //2
      0, // Integral Gain //10
      0, // Derivative Gain //0
      0.0, // Cage Limit //0.3 //0.1 //0.2
      0.0, // Deadband //0
      2.5,// MaxOutput Meters/sec 0.25 //100 //1
      true, //enableCage
      false //enableDeadband
    );

    // Heading Command PID for Autonomous and 
    headingPID.initialize2(
      4, // Proportional Gain // 2
      0, // Integral Gain // 5
      0.0, // Derivative Gain 
      10, // Cage Limit //0.3
      0.0, // Deadband
      1000, // MaxOutput Degrees/sec 0.25 //100 //180
      false, //enableCage
      false //enableDeadband
    );

    limelightRotationPID.initialize2(
      0.02, // Proportional Gain 0.02=without weights, 0.03=with weights
      0.015, // Integral Gain 0.05=without weights 0.015= with weights
      0.0, // Derivative Gain -0.0008 =without weights, with weights
      2, // Cage Limit degrees/sec 2=without weights, with weights
      0.0, // Deadband
      0.4, // MaxOutput Degrees/sec 
      true, //enableCage
      false //enableDeadband
    );
  }

  public void stop() {
    drive(0, 0);
  }

  public void drive(double throttle, double rotate) {
    leftBackMotor.set(throttle - rotate);
    rightBackMotor.set(throttle + rotate);
  }

  public void setModePercentVoltage() {
    leftFrontMotor.set(ControlMode.PercentOutput, 0);
    rightFrontMotor.set(ControlMode.PercentOutput, 0);
    leftBackMotor.set(ControlMode.PercentOutput, 0);
    rightBackMotor.set(ControlMode.PercentOutput, 0);
  }

  public double distanceTravelledinTicks() {
    return (getBackLeftEncoderPosition() + getBackRightEncoderPosition()) / 2;
  }

  public double getBackLeftEncoderPosition() {
    return leftBackMotor.getSelectedSensorPosition();
  }

  public double getBackRightEncoderPosition() {
    return rightBackMotor.getSelectedSensorPosition();
  }

  public double getBackLeftEncoderVelocity() {
    return leftBackMotor.getSelectedSensorVelocity();
  }

  public double getBackRightEncoderVelocity() {
    return rightBackMotor.getSelectedSensorVelocity();
  }

  public double getBackLeftEncoderVelocityMetersPerSecond() {
    // getQuadVelocity is in 100 ms so we have to divide it by 10 to get seconds
    double backLeftVelocityMPS = (leftBackMotor.getSelectedSensorVelocity() * 10); // /10
    // since getQuadVelocity is in encoder ticks, we have to convert it to meters
    backLeftVelocityMPS = backLeftVelocityMPS * METERS_PER_TICKS;
    return (backLeftVelocityMPS);
  }

  public double getBackRightEncoderVelocityMetersPerSecond() {
    // getQuadVelocity is in 100 ms so we have to divide it by 10 to get seconds
    double backRightVelocityMPS = (rightBackMotor.getSelectedSensorVelocity() * 10); // /10
    // since getQuadVelocity is in encoder ticks, we have to convert it to meters
    // Need to have a negative for right velocity since the motors are reversed on
    // the opposite side
    backRightVelocityMPS = backRightVelocityMPS * METERS_PER_TICKS;
    return (backRightVelocityMPS);
  }

  public double getAverageEncoderVelocityMetersPerSecond() {
    double velocityMPS = (getBackLeftEncoderVelocityMetersPerSecond() + getBackRightEncoderVelocityMetersPerSecond())
        * 0.5;
    return (velocityMPS);
  }

  public double leftDistanceTravelledInMeters() {
    double left_dist = getBackLeftEncoderPosition() * METERS_PER_TICKS;
    return left_dist;
  }

  public double rightDistanceTravelledInMeters() {
    double right_dist = getBackRightEncoderPosition() * METERS_PER_TICKS;
    return right_dist;
  }

  public double distanceTravelledinMeters() {
    // left distance is negative because the encoder value on the 
    // left is negative when dev bot is pushed forward 2/15/20
    // Code Tested on Dev Bot, Works on 2/15/20
    double distanceTravelled = (leftDistanceTravelledInMeters() + rightDistanceTravelledInMeters()) / 2;
    return distanceTravelled;
  }

  public void resetEncoders() {
    leftBackMotor.setSelectedSensorPosition(0);
    rightBackMotor.setSelectedSensorPosition(0);
    leftFrontMotor.setSelectedSensorPosition(0);
    rightFrontMotor.setSelectedSensorPosition(0);
  }

  public double limit_output(double motorCommandValue) {

    if (motorCommandValue > 1) {
      motorCommandValue = 1;
    }
    else if (motorCommandValue < -1) {
      motorCommandValue = -1;
    }

    return motorCommandValue;
  }

  // feedforward calculation
  public double calculateNew(double velocity, double acceleration, double ks, double kv, double ka) {
    return ks * Math.signum(velocity) + kv * velocity + ka * acceleration;
  } 

  // parameters left/rightSpeed are meters per second
  public void straightDrive(double leftSpeed, double rightSpeed) {

    left_speed_feedback = getBackLeftEncoderVelocityMetersPerSecond();
    right_speed_feedback = getBackRightEncoderVelocityMetersPerSecond();

    // actual speed command passed 
    left_speed_cmd = leftSpeed; // m/s
    right_speed_cmd = rightSpeed; // m/s
    
    // calculate rate feedforward term in meters per second
    final double leftFeedforward = calculateNew(leftSpeed, 0, 0.7*0.8, 2.5*1.15*0.95*1.1, 0); //0.7*0.8, 2.5*1.15*0.95*1.1
    final double rightFeedforward = calculateNew(rightSpeed, 0, 0.7*0.8, 2.5*0.95*1.1, 0); //0.7*0.8, 2.5*0.95*1.1

    double batteryVoltage = RobotController.getBatteryVoltage(); // getting battery voltage from PDP via the rio

    // we do this to make sure our calculations do not go too high and prevent undef (divided by 0)
    if (batteryVoltage < 1) {
      batteryVoltage = 1;
    }
    
    // output to compensate for speed error, the PID block
    double leftOutput = leftForwardPID.execute(leftSpeed, left_speed_feedback);
    double rightOutput = rightForwardPID.execute(rightSpeed, right_speed_feedback);

    // double torque_bias = 0.1;

    // this was meant to counter the penguin hop by lowering gain on both sides
    /*if (Math.abs(leftShooterMotor.getSelectedSensorVelocity()) > 0.1) {
      leftOutput = leftForwardPID.execute(leftSpeed, left_speed_feedback);
      rightOutput = rightForwardPID.execute(rightSpeed, right_speed_feedback);
    } else {
      leftOutput = 0;
      rightOutput = 0;
    }*/

    /*
    if (driverController.getBButton()) {
      leftOutput = leftForwardPID.execute(leftSpeed, getBackLeftEncoderVelocityMetersPerSecond());
      rightOutput = rightForwardPID.execute(rightSpeed, getBackRightEncoderVelocityMetersPerSecond());
    } else {
      leftOutput = 0;
      rightOutput = 0;

    }*/

    /*leftOutput = 0;
    rightOutput = 0;*/

    // final voltage command going to falcon or talon (percent voltage, max 12 V)
    double LVoltagePercentCommand = ((leftOutput + leftFeedforward) / batteryVoltage); //((leftFeedforward) / batteryVoltage); 
    double RVoltagePercentCommand =  ((rightOutput + rightFeedforward) / batteryVoltage); //((rightFeedforward) / batteryVoltage);

    // should never have value above 1 or -1, always inbetween
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

    leftBackMotor.set(LVoltagePercentCommand);
    rightBackMotor.set(RVoltagePercentCommand);

    /*
    double left_front_motor_command = 0;
    double left_back_motor_command = 0;
    double right_front_motor_command = 0;
    double right_back_motor_command = 0;
    */

    //compute the command to each motor to use torque bias
    /*
    if (Math.abs(LVoltagePercentCommand) > 0.1){
      left_front_motor_command = LVoltagePercentCommand + torque_bias;
      left_back_motor_command = LVoltagePercentCommand - torque_bias;
    } else {
      left_front_motor_command = 0;
      left_back_motor_command = 0;
    }
    if (Math.abs(RVoltagePercentCommand) > 0.1){ 
      right_front_motor_command = RVoltagePercentCommand + torque_bias;
      right_back_motor_command = RVoltagePercentCommand - torque_bias;
    } else {
      right_front_motor_command = 0;
      right_back_motor_command = 0;
    }
    */

    //limit the commands… its best to use a function, limit the min and max to +/-1.0 bad behavior happens if you don’t
    /*
    left_front_motor_command = limit_output(left_front_motor_command);
    left_back_motor_command = limit_output(left_back_motor_command);
    right_front_motor_command = limit_output(right_front_motor_command);
    right_back_motor_command = limit_output(right_back_motor_command);
    */

    //now command each motor individually
    /*
    leftFrontMotor.set(left_front_motor_command);
    leftBackMotor.set(left_back_motor_command);
    rightFrontMotor.set(right_front_motor_command);
    rightBackMotor.set(right_back_motor_command);
    */

    //System.out.println("Left Error:" + (leftSpeed - getBackLeftEncoderVelocityMetersPerSecond()));
    //System.out.println(getLeftEncoderVelocityMetersPerSecond() + "," + getRightEncoderVelocityMetersPerSecond());
    // System.out.println("Left Error: " + (leftSpeed-getLeftEncoderVelocityMetersPerSecond()) + "/ Right Error: " + (getRightEncoderVelocityMetersPerSecond()-rightSpeed));
    // System.out.println("Debug Out  " + rightOutput + " /// " + rightFeedforward + " /// " + JoystickDrive.throttle);
  }

  public void driveWithRotation(double linearTravelSpeed, double rotateSpeed) {
    // input speed is meters per second, input rotation is bot rotation 
    // speed in meters per second

    double leftSpeed = 0; // meters per second
    double rightSpeed = 0; // meters per second

    // Steps:
    // 1 - decide accel or decel rn
    // 2 - limit commanded velocity based on computed accel limit

    double linearAccel = (linearTravelSpeed - lastLinearVelocity)/0.02;
    double rotateAccel = (rotateSpeed - lastRotateVelocity)/0.02;

    double accelLimit = 1; //meters per second

    // are we accel or decel? part 1
    if (lastLinearVelocity > 0) {
      if (linearAccel < 0) {
        // velocity pos, accel negative, speed dec (slowing down), use decel
        accelLimit = 2; // meters per second
      } else {
        accelLimit = 1;
      }
    } else { // we have negative velocity command
      if (linearAccel > 0) {
        // velocity neg, accel pos, speed dec, decel
        accelLimit = 2;
      } else {
        accelLimit = 1;
      }
    }

    if (lastRotateVelocity > 0) {
      if (rotateAccel < 0) {
        // velocity pos, accel negative, speed dec (slowing down), use decel
        accelLimit = 2; // meters per second
      } else {
        accelLimit = 1;
      }
    } else { // we have negative velocity command
      if (rotateAccel > 0) {
        // velocity neg, accel pos, speed dec, decel
        accelLimit = 2;
      } else {
        accelLimit = 1;
      }
    }

    //System.out.println("Linear accel: " + linearAccel);
  
    // part 2: limit velocity based on accelLimit
    if (linearAccel > accelLimit) {
      linearTravelSpeed = lastLinearVelocity + accelLimit*0.03;
    }
    else if (linearAccel < -accelLimit) {
      linearTravelSpeed = lastLinearVelocity - accelLimit*0.03;
    }

    if (rotateAccel > accelLimit) {
      rotateSpeed = lastRotateVelocity + accelLimit*0.03;
    }
    else if (rotateAccel < -accelLimit) {
      rotateSpeed = lastRotateVelocity - accelLimit*0.03;
    }

    lastLinearVelocity = linearTravelSpeed; //meters per second
    lastRotateVelocity = rotateSpeed;
    //System.out.println("Linear travel speed: " + linearTravelSpeed);

    // this block is helping fix the "penguin hop" of the drivetrain
    // operates on input from joystick
    // pt. 2 A
    if (Math.abs(linearTravelSpeed) < 1.5 && linearTravelSpeed > 0.05) {
      
      if (rotateSpeed > 0.3) {
        leftSpeed = (linearTravelSpeed);
        rightSpeed = (linearTravelSpeed - rotateSpeed);
        //leftSpeed = (linearTravelSpeed + rotateSpeed);
        //rightSpeed = (linearTravelSpeed);
      } else  if (rotateSpeed < -0.3) {
        leftSpeed = (linearTravelSpeed + rotateSpeed);
        rightSpeed = (linearTravelSpeed);
        //leftSpeed = (linearTravelSpeed);
        //rightSpeed = (linearTravelSpeed - rotateSpeed);
      } else {
        // original way to do tank drive before trying to get rid of penguin
        leftSpeed = (linearTravelSpeed + rotateSpeed); 
        rightSpeed = (linearTravelSpeed - rotateSpeed);
      }
    } else {
      // original way to do tank drive before trying to get rid of penguin
      leftSpeed = (linearTravelSpeed + rotateSpeed);
      rightSpeed = (linearTravelSpeed - rotateSpeed);
    }

    // pt. 2 B
    // now look at ground speed when moving forward. if we get rotate cmd that is positive, 
    // the left speed should increase (go positive) while the right speed should go negative
    // conversely, if we have a negative rotatation, the left speed should be negative (dec speed) and 
    // the right speed should be positive (inc speed)
    /*if (rotateSpeed > 0.1) { // this means we are trying to rotate right
      // pt. 2 B1 - looking at forward, positive direction movement
      if (right_speed_feedback > 0.05) { // reading the right speed feedback as positive
        if (right_speed_cmd < 0) { // cmd moving in the opposite direction
          // we think this is the condition that causes the skip (moving forward, but turn in opposite direction)
          // recompute right speed = travel speed
          rightSpeed = linearTravelSpeed;
          //System.out.println("PENGUIN HOP HAPPENING");
        }
      }
      if (left_speed_feedback > 0.05) { // reading the left speed feedback as positive
        if (left_speed_cmd < 0) { // cmd moving in the opposite direction
          // we think this is the condition that causes the skip (moving forward, but turn in opposite direction)
          // recompute left speed = travel speed
          leftSpeed = linearTravelSpeed;
          //System.out.println("PENGUIN HOP HAPPENING");
        }
      }
      // pt. 2 B2 - moving backwards, negative direction
      if (right_speed_feedback < -0.05) { // reading the right speed feedback as negative
        if (right_speed_cmd > 0) { // cmd moving in the opposite direction
          // we think this is the condition that causes the skip (moving forward, but turn in opposite direction)
          // recompute right speed = travel speed
          rightSpeed = linearTravelSpeed;
          //System.out.println("PENGUIN HOP HAPPENING");
        }
      }
      if (left_speed_feedback < -0.05) { // reading the left speed feedback as negative
        if (left_speed_cmd > 0) { // cmd moving in the opposite direction
          // we think this is the condition that causes the skip (moving forward, but turn in opposite direction)
          // recompute left speed = travel speed
          leftSpeed = linearTravelSpeed;
          //System.out.println("PENGUIN HOP HAPPENING");
        }
      }
    } 

    if (rotateSpeed < -0.1) { // we are rotating left
      // pt. 2 B1 - looking at forward, positive direction movement
      if (left_speed_feedback > 0.05) { // reading the left speed feedback as positive
        if (left_speed_cmd < 0) { // cmd moving in the opposite direction
          // we think this is the condition that causes the skip (moving forward, but turn in opposite direction)
          // recompute left speed = travel speed
          leftSpeed = linearTravelSpeed;
          //System.out.println("PENGUIN HOP HAPPENING");
        }
      } 
      if (right_speed_feedback > 0.05) { // reading the right speed feedback as positive
        if (right_speed_cmd < 0) { // cmd moving in the opposite direction
          // we think this is the condition that causes the skip (moving forward, but turn in opposite direction)
          // recompute left speed = travel speed
          rightSpeed = linearTravelSpeed;
          //System.out.println("PENGUIN HOP HAPPENING");
        }
      } 
      // pt. 2 B2 - moving backwards, negative direction
      if (right_speed_feedback < -0.05) { // reading the right speed feedback as negative
        if (right_speed_cmd > 0) { // cmd moving in the opposite direction
          // we think this is the condition that causes the skip (moving forward, but turn in opposite direction)
          // recompute right speed = travel speed
          rightSpeed = linearTravelSpeed;
          //System.out.println("PENGUIN HOP HAPPENING");
        }
      }
      if (left_speed_feedback < -0.05) { // reading the left speed feedback as negative
        if (left_speed_cmd > 0) { // cmd moving in the opposite direction
          // we think this is the condition that causes the skip (moving forward, but turn in opposite direction)
          // recompute left speed = travel speed
          leftSpeed = linearTravelSpeed;
          //System.out.println("PENGUIN HOP HAPPENING");
        }
      }
    }*/

    // part 3: reset speeds to 0 when throttle and rotate < 0.25
    /*if ((Math.abs(linearTravelSpeed) < 0.5) && 
        (Math.abs(rotateSpeed) < 0.5)) {
      leftSpeed = 0;
      rightSpeed = 0;
      lastVelocityLeft = 0;
      lastVelocityRight = 0;
      lastLinearVelocity = 0;
    }*/

    straightDrive(leftSpeed, rightSpeed);
  }

  public void autoDrive(double distance, double heading) { // distance is in meters, heading is in degrees
    // double left_speed;
    // double right_speed;                                                                                                                                                                                                                                         
    double start_position = distanceTravelledinMeters();
    if(state_flag_motion_profile) { // if flag is true, needs to be restarting everything
        positionMotionProfile.init(
          start_position, //start position
          distance + start_position, // target position
          2.4, // max vel //1.5 // 1 //1.2
          1.2, // max accel //1 // 0.5
          0.02, // execution period 
          1.2 // deceleration //2 // 0.5
        );
        state_flag_motion_profile = false;
    }

    double position_profile_command = positionMotionProfile.execute();
    double feed_forward_rate = positionMotionProfile.velocity_feed_forward;


    //System.out.println(feed_forward_rate);

    double headingFeedback = getYaw(); // in degrees
    double headingCommand = heading;
    double headingError = headingPID.execute(headingCommand, headingFeedback);
    double headingErrorMeters = HEADING_BOT_DEG_TO_BOT_WHEEL_DISTANCE * headingError;

    //System.out.println(headingCommand + "," + headingFeedback);

    double position_feedback = distanceTravelledinMeters();
    
    //SmartDashboard.putNumber("Auto Distance", position_feedback);
    // positionError is in meters per second
    double positionError = positionPID.execute(position_profile_command, position_feedback);
    double positionCmdOut = (positionError+feed_forward_rate);

    //System.out.println("position profile command: " + position_profile_command + ", " + 
    //"bot position from encoder: " + position_feedback + ", PID output for position: " + positionError);

    // System.out.println("Cmd: " + position_profile_command + " Fb: " + position_feedback + " Vel: " + getAverageEncoderVelocityMetersPerSecond());
    // System.out.println("Feedback: " + position_feedback);
    // System.out.println("PID Error: " + positionError);
    //System.out.println("\n");

    // left_speed = output;
    // right_speed = output;

    // Refer to the rate drive control diagram
    // We modulate our speed of the bot to close out the position error, making it eventually zero
    driveWithRotation(positionCmdOut, headingErrorMeters);
    //driveWithRotation(0.0, headingErrorMeters);
    //driveWithRotation(positionError, 0);
    //System.out.println("Pos " + position_feedback + " PE " + positionError);
    // System.out.println("TD " + distance + " // DT " + position_feedback);
  }

  // 4/13/22 keep the forward/backward movement separate from rotation
  public void autoRotate(double finalHeading) { // finalHeading is in degrees
    // double left_speed;
    // double right_speed;                                                                                                                                                                                                                                         
    double headingFeedback = getYaw(); // in degrees

    if(state_flag_motion_profile) { // if flag is true, needs to be restarting everything
        /*positionMotionProfile.init(
          start_position, //start position
          distance + start_position, // target position
          1.2, // max vel //1.5 // 1 //1.2
          1.2, // max accel //1 // 0.5
          0.02, // execution period 
          1.2 // deceleration //2 // 0.5
        );*/
        //double start_position = distanceTravelledinMeters();

        headingMotionProfile.init(
          headingFeedback, //starting heading
          headingFeedback + finalHeading, // final heading
          360, // max vel (in degrees per sec)
          360, // max accel (degrees per sec squared)
          0.02, // execution period 
          360 // deceleration
        );
        state_flag_motion_profile = false;
    }

    /*double position_profile_command = positionMotionProfile.execute();
    double feed_forward_rate = 0; //positionMotionProfile.velocity_feed_forward;*/
    double heading_profile_command = headingMotionProfile.execute();

    //double heading_feed_forward_rate = headingMotionProfile.velocity_feed_forward;

    //System.out.println(feed_forward_rate);

    
    //double headingCommand = finalHeading;
    double headingError = headingPID.execute(heading_profile_command, headingFeedback);
    double headingErrorMeters = HEADING_BOT_DEG_TO_BOT_WHEEL_DISTANCE * headingError;//+heading_feed_forward_rate);

    //System.out.println(headingCommand + "," + headingFeedback);

    //double position_feedback = distanceTravelledinMeters();
    
    //SmartDashboard.putNumber("Auto Distance", position_feedback);
    // positionError is in meters per second
    //double positionError = positionPID.execute(start_position, position_feedback);
    //double positionCmdOut = (positionError+feed_forward_rate);

    //System.out.println("position profile command: " + position_profile_command + ", " + 
    //"bot position from encoder: " + position_feedback + ", PID output for position: " + positionError);

    // System.out.println("Cmd: " + position_profile_command + " Fb: " + position_feedback + " Vel: " + getAverageEncoderVelocityMetersPerSecond());
    // System.out.println("Feedback: " + position_feedback);
    // System.out.println("PID Error: " + positionError);
    //System.out.println("\n");

    // left_speed = output;
    // right_speed = output;

    // Refer to the rate drive control diagram
    // We modulate our speed of the bot to close out the position error, making it eventually zero
    driveWithRotation(0, headingErrorMeters); // don't want to move forward/backward, just rotate in place
    //driveWithRotation(0.0, headingErrorMeters);
    //driveWithRotation(positionError, 0);
    //System.out.println("Pos " + position_feedback + " PE " + positionError);
    // System.out.println("TD " + distance + " // DT " + position_feedback);
  }

  public double getYaw() {
    double yaw = imu.getYaw();
    return yaw;
  }

  public double getPitch() {
    double pitch = imu.getPitch();
    return pitch;
  }

  public double getRoll() {
    double roll = imu.getRoll();
    return roll;
  }

  /*
  public double[] getRPH() {
    double[] ypr = new double[3];
    imu.getYawPitchRoll(ypr);
    ypr[0] = -ypr[0];
    return(ypr);
  }*/

  public void ZeroYaw() {
    imu.setYaw(0, 10);
    //imu.setFusedHeading(0, 10);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}