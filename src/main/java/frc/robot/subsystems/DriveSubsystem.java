// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
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
  
  /* Every Bot Variables */
  private static final int timeoutMs = 10;
  private static final int MOTOR_ENCODER_CODES_PER_REV = 2048; //4096 for CTRE Mag Encoders, 2048 for the Falcons
  private static final double DIAMETER_INCHES = 5.0; // Flex wheels on Everybot
  
  private static final double WHEEL_DIAMETER = DIAMETER_INCHES * IN_TO_M; // in meters
  private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
  // This is for a output gear side motor encoder
  // private static final double TICKS_PER_METER = MOTOR_ENCODER_CODES_PER_REV / WHEEL_CIRCUMFERENCE;

  private static final double GEAR_RATIO = 10.71;

  // This is for an encoder mounted to the motor
  private static final double TICKS_PER_METER = (MOTOR_ENCODER_CODES_PER_REV * GEAR_RATIO) / (WHEEL_CIRCUMFERENCE);
  private static final double METERS_PER_TICKS = 1 / TICKS_PER_METER;
  private static final double BOT_WHEEL_TO_WHEEL_DIAMETER = 0.49;//METERS

  public boolean state_flag_motion_profile = true;

  //90 degrees /360 = 2*PI*R 
  private static final double HEADING_BOT_DEG_TO_BOT_WHEEL_DISTANCE = (BOT_WHEEL_TO_WHEEL_DIAMETER * Math.PI)/360.0;

  private static final WPI_TalonFX leftBackMotor = RobotMap.leftBackDriveMotor;
  private static final WPI_TalonFX rightBackMotor = RobotMap.rightBackDriveMotor;
  private static final WPI_TalonFX leftFrontMotor = RobotMap.leftFrontDriveMotor;
  private static final WPI_TalonFX rightFrontMotor = RobotMap.rightFrontDriveMotor;

  private static XboxController driverController = RobotContainer.driverController;

  //private final PigeonIMU imu = RobotMap.drive_imu;

  public RoboLionsPID leftForwardPID = new RoboLionsPID();
  public RoboLionsPID rightForwardPID = new RoboLionsPID();
	public RoboLionsPID headingPID = new RoboLionsPID();
  //public RoboLionsPID limelightPID = new RoboLionsPID();
  public RoboLionsPID positionPID = new RoboLionsPID();
  //public RoboLionsMotionProfile positionMotionProfile = new RoboLionsMotionProfile();
  //public RoboLionsMotionProfile headingMotionProfile = new RoboLionsMotionProfile();

  public double left_speed_cmd;
  public double right_speed_cmd;

  /*static double lastVelocityLeft = 0;
  static double lastVelocityRight = 0;*/

  static double lastLinearVelocity = 0;

/*
  static double lastLinearVelocityLeft = 0;
  static double lastLinearVelocityRight = 0;
  static double lastRotateVelocityLeft = 0;
  static double lastRotateVelocityRight = 0;
  */

  /*
  public static final double LLAIMINGCONSTANT = 0; //  limelight aiming constant TODO: change
  public static final double MOTORGAIN = 0; //TODO: change
  public double LX = LimelightSubsystem.limelight_x;*/
  
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    //ZeroYaw();
    resetEncoders();

    leftFrontMotor.set(ControlMode.Follower, leftBackMotor.getDeviceID());
    rightFrontMotor.set(ControlMode.Follower, rightBackMotor.getDeviceID());

    leftBackMotor.setNeutralMode(NeutralMode.Coast);
    rightBackMotor.setNeutralMode(NeutralMode.Coast);

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
        // 0.5 = 2.94, 28, 0.077
        3.375, // Proportional Gain 4.9, 7.5, 2.205, 3.375
        //.21 seconds
        17.357*0.1, // Integral Gain //12.6 42.12 ZN w FF, 17.357
        0.0, // Derivative Gain //0
        0.0, // Cage Limit 0.3 //0
        0.0, // Deadband //0
        12,// MaxOutput Volts 0.25 //100 //12
        false, //enableCage
        false //enableDeadband
    );

    // Rate Drive PID
    rightForwardPID.initialize2(
        3.15, // 2.025 Proportional Gain 4.5, 7 //2.925 ZN w FF //2
        16.2*0.1, // 10.2316 Integral Gain //42.12 ZN w FF //20
        0, // Derivative Gain //0
        0.0, // Cage Limit //0.3
        0.0, // Deadband //0
        12,// MaxOutput Volts 0.25 //100 //12
        false, //enableCage
        false //enableDeadband
    );

    // Position Command PID for Autonomous and 
    positionPID.initialize2(
        0, // Proportional Gain //1.35 //2
        0, // Integral Gain //5 //10
        0.0, // Derivative Gain //0
        0.0, // Cage Limit //0.3 //0.1 //0.2
        0.0, // Deadband //0
        2.5,// MaxOutput Meters/sec 0.25 //100 //1
        true, //enableCage
        false //enableDeadband
    );

    // Heading Command PID for Autonomous and 
    headingPID.initialize2(
        0, // Proportional Gain //15 // 7.5
        0.0, // Integral Gain // 10
        0.0, // Derivative Gain 
        20, // Cage Limit //0.3
        0.0, // Deadband
        360, // MaxOutput Degrees/sec 0.25 //100 //180
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

  // feedforward calculation
  public double calculateNew(double velocity, double acceleration, double ks, double kv, double ka) {
    return ks * Math.signum(velocity) + kv * velocity + ka * acceleration;
  } 

  public void straightDrive(double leftSpeed, double rightSpeed) {
    // TODO: change
    left_speed_cmd = leftSpeed; // actual speed command passed 
    right_speed_cmd = rightSpeed;
    
    // calculate rate feedforward term
    final double leftFeedforward = calculateNew(leftSpeed, 0, 0.7*0.8, 2.5*1.15*0.95*1.1, 0); //0.69, 2.43
    final double rightFeedforward = calculateNew(rightSpeed, 0, 0.7*0.8, 2.5*0.95*1.1, 0); //2.3

    double batteryVoltage = RobotController.getBatteryVoltage(); // getting battery voltage from PDP via the rio

    if (batteryVoltage < 1) {
        batteryVoltage = 1;
    }

    // output to compensate for speed error, the PID block
    double leftOutput = leftForwardPID.execute(leftSpeed, getBackLeftEncoderVelocityMetersPerSecond());
    double rightOutput = rightForwardPID.execute(rightSpeed, getBackRightEncoderVelocityMetersPerSecond());
    
    // final voltage command going to falcon or talon (percent voltage, max 12 V)
    /*double LVoltagePercentCommand = ((leftOutput + leftFeedforward) / batteryVoltage);
    double RVoltagePercentCommand = ((rightOutput + rightFeedforward) / batteryVoltage);*/

    //eliminate jerk test
    double LVoltagePercentCommand = ((leftFeedforward) / batteryVoltage);
    double RVoltagePercentCommand = ((rightFeedforward) / batteryVoltage);

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

    SmartDashboard.putNumber("Right Motor Command", RVoltagePercentCommand);
    //SmartDashboard.putNumber("Left Motor Command", LVoltagePercentCommand);
    
    leftBackMotor.set(LVoltagePercentCommand);
    rightBackMotor.set(RVoltagePercentCommand);

    SmartDashboard.putNumber("leftSpeed", leftSpeed);
    //System.out.println("Left speed: " + leftSpeed);
    SmartDashboard.putNumber("rightSpeed", rightSpeed);
    
    //SmartDashboard.putNumber("Yaw Value", getYaw());
    //SmartDashboard.putNumber("Distance Travelled", distanceTravelledinMeters());
    //SmartDashboard.putNumber("Left Encoder Counts", getLeftEncoderPosition());
    //SmartDashboard.putNumber("Right Encoder Counts", getRightEncoderPosition());
    
    SmartDashboard.putNumber("Left Dist Meters", leftDistanceTravelledInMeters());
    SmartDashboard.putNumber("Right Dist Meters", rightDistanceTravelledInMeters());

    SmartDashboard.putNumber("Left Encoder MPS", getBackLeftEncoderVelocityMetersPerSecond());
    SmartDashboard.putNumber("Right Encoder MPS", getBackRightEncoderVelocityMetersPerSecond());
    
    //System.out.println("Left Error:" + (leftSpeed - getBackLeftEncoderVelocityMetersPerSecond()));
    //System.out.println(getLeftEncoderVelocityMetersPerSecond() + "," + getRightEncoderVelocityMetersPerSecond());
    // System.out.println("Left Error: " + (leftSpeed-getLeftEncoderVelocityMetersPerSecond()) + "/ Right Error: " + (getRightEncoderVelocityMetersPerSecond()-rightSpeed));
    // System.out.println("Debug Out  " + rightOutput + " /// " + rightFeedforward + " /// " + JoystickDrive.throttle);
  }

  public void driveWithRotation(double linearTravelSpeed, double rotateSpeed) {
    // input speed is meters per second, input rotation is bot rotation 
    // speed in meters per second
    // dev bot requires the output to be inverted, everybot needs it to NOT be inverted

    double leftSpeed = 0;
    double rightSpeed = 0;

    // reset speeds to 0 when throttle and rotate < 0.25
    if ((Math.abs(linearTravelSpeed) < 0.25) && 
        (Math.abs(rotateSpeed) < 0.25)) {
      leftSpeed = 0;
      rightSpeed = 0;
      /*lastVelocityLeft = 0;
      lastVelocityRight = 0;*/
      lastLinearVelocity = 0;
    }

    double linearAccel = (linearTravelSpeed - lastLinearVelocity)/0.02;
    //System.out.println("Linear accel: " + linearAccel);
  
    double accelLimit = 1; //meters per second
    if (linearAccel > accelLimit) {
      linearTravelSpeed = lastLinearVelocity + accelLimit*0.01;
    }
    else if (linearAccel < -accelLimit) {
      linearTravelSpeed = lastLinearVelocity - accelLimit*0.01;
    }

    lastLinearVelocity = linearTravelSpeed;
    //System.out.println("Linear travel speed: " + linearTravelSpeed);

    leftSpeed = (linearTravelSpeed + rotateSpeed);
    rightSpeed = (linearTravelSpeed - rotateSpeed);

    /*
    // acceleration concept:
    double leftAccel = (leftSpeed - lastVelocityLeft)/0.02;
    double rightAccel = (rightSpeed - lastVelocityRight)/0.02;

    if (leftAccel > accelLimit) {
      leftSpeed = lastVelocityLeft + accelLimit*0.02;
    } 
    else if (leftAccel < -accelLimit) {
      leftSpeed = lastVelocityLeft - accelLimit*0.02;
    }

    if (rightAccel > accelLimit) {
      rightSpeed = lastVelocityRight + accelLimit*0.02;
    } 
    else if (rightAccel < -accelLimit) {
      rightSpeed = lastVelocityRight - accelLimit*0.02;
    }

    lastVelocityLeft = leftSpeed;
    lastVelocityRight = rightSpeed;*/

    straightDrive(leftSpeed, rightSpeed);
  }

  /*
  public void autoDrive(double distance, double heading) { // distance is in meters, heading is in degrees
    double start_dist = distanceTravelledinMeters();
    if(state_flag_motion_profile) {
        positionMotionProfile.init(
          start_dist, //start position
          distance, // target position
          1, // max vel //1.5 // 1
          1, // max accel //1 // 0.5
          0.02, // execution period 
          1 // deceleration //2 // 0.5
        );
        state_flag_motion_profile = false;
    }

    double position_profile_command = positionMotionProfile.execute();
    double feed_forward_rate = positionMotionProfile.velocity_feed_forward;

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

    // System.out.println("Cmd: " + position_profile_command + " Fb: " + position_feedback + " Vel: " + getAverageEncoderVelocityMetersPerSecond());
    // System.out.println("Feedback: " + position_feedback);
    // System.out.println("PID Error: " + positionError);
    //System.out.println("\n");

    // left_speed = output;
    // right_speed = output;

    // straightDrive(left_speed, right_speed);
    // Refer to the rate drive control diagram
    // We modulate our speed of the bot to close out
    // the position error, making it eventually zero
    driveWithRotation(positionCmdOut, headingErrorMeters);
    //driveWithRotation(0.0, headingErrorMeters);
    //driveWithRotation(positionError, 0);
    // riveWithRotation(0.5, 0.0);
    //System.out.println("Pos " + position_feedback + " PE " + positionError);
    // System.out.println("TD " + distance + " // DT " + position_feedback);
  }*/

  /*
  public double getYaw() {
    double[] ypr = new double[3];
    imu.getYawPitchRoll(ypr);
    return -ypr[0];
  }

  public double getPitch() {
    double[] ypr = new double[3];
    imu.getYawPitchRoll(ypr);
    return ypr[1];
  }

  public double getRoll() {
    double[] ypr = new double[3];
    imu.getYawPitchRoll(ypr);
    return ypr[2];
  }


  public double[] getRPH() {
    double[] ypr = new double[3];
    imu.getYawPitchRoll(ypr);
    ypr[0] = -ypr[0];
    return(ypr);
  }

  public void ZeroYaw() {
    imu.setYaw(0, 10);
    imu.setFusedHeading(0, 10);
  }*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}