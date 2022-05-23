// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimbSubsystem;

public class MoveClimb extends CommandBase {

  public static final double LEFT_UP_POWER = 1; 
  public static final double RIGHT_UP_POWER = -1;

  public static final double LEFT_DOWN_POWER = -1;
  public static final double RIGHT_DOWN_POWER = 1; 

  public static final double LEFT_SLOW_UP_POWER = 0.2;
  public static final double LEFT_SLOW_DOWN_POWER = -0.2;
  public static final double RIGHT_SLOW_UP_POWER = -0.2;
  public static final double RIGHT_SLOW_DOWN_POWER = 0.2;

  public static final double LEFT_TEST_UP_POWER = 0.1;
  public static final double LEFT_TEST_DOWN_POWER = -0.1;
  public static final double RIGHT_TEST_UP_POWER = -0.1;
  public static final double RIGHT_TEST_DOWN_POWER = 0.1;

  public static final double R_MAX_ENCODER_COUNT = 9000000;
  public static final double R_MIN_ENCODER_COUNT = 0;
  public static final double R_MID_TARGET_ENCODER_COUNT = 9000000; 
  public static final double R_CLIMB_TARGET_ENCODER_COUNT = 0;

  public static final double L_MAX_ENCODER_COUNT = 90000000;
  public static final double L_MIN_ENCODER_COUNT = 0;
  public static final double L_MID_TARGET_ENCODER_COUNT = 9000000;
  public static final double L_CLIMB_TARGET_ENCODER_COUNT = 0;

  private final ClimbSubsystem climbSubsystem;
  private final XboxController driverController = RobotContainer.driverController;

  public static int climb_motion_state = 0;

  public static double leftClimbPower = 0;
  public static double rightClimbPower = 0;
  public static double rightStartingPosition;
  public static double leftStartingPosition;

  public MoveClimb(ClimbSubsystem climb) {
    climbSubsystem = climb;
    addRequirements(climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //climbSubsystem.stopClimb();
    rightStartingPosition = Math.abs(climbSubsystem.getRightEncoderPosition());
    leftStartingPosition = Math.abs(climbSubsystem.getLeftEncoderPosition());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rightCurrentPosition = Math.abs(climbSubsystem.getRightEncoderPosition());
    double leftCurrentPosition = Math.abs(climbSubsystem.getLeftEncoderPosition());

    //System.out.println("Right Current Position: " + rightCurrentPosition);
    //System.out.println("Left Current Position: " + leftCurrentPosition);
    
    boolean left_bumper = driverController.getLeftBumper();
    boolean right_bumper = driverController.getRightBumper();

    boolean start_button = driverController.getStartButton();
    boolean back_button = driverController.getBackButton();

    if ((climbSubsystem.getRightLimitSwitchValue() == 1) || (climbSubsystem.getLeftLimitSwitchValue() == 1)) {
      climbSubsystem.resetEncoder();
    }

    // Pull down climber DURING COMPETITION (climbing)
    /*if (left_bumper && 
       (rightCurrentPosition > R_CLIMB_TARGET_ENCODER_COUNT) && 
       (leftCurrentPosition > L_CLIMB_TARGET_ENCODER_COUNT)) {
      leftClimbPower = LEFT_DOWN_POWER; // moving inwards
      rightClimbPower = RIGHT_DOWN_POWER;
    } */
    // Pull up climber to target position DURING COMPETITION
   /* else if (right_bumper && 
            (rightCurrentPosition < R_MID_TARGET_ENCODER_COUNT) &&
            (leftCurrentPosition < L_MID_TARGET_ENCODER_COUNT)) {
      leftClimbPower = LEFT_UP_POWER;
      rightClimbPower = RIGHT_UP_POWER;
    } */
    // Pull down climber to reset (home)
    else if (back_button) {
      leftClimbPower = LEFT_SLOW_DOWN_POWER;
      rightClimbPower = RIGHT_SLOW_DOWN_POWER;
    }
    // Pull up climber to max position
    else if (start_button) {
      climbSubsystem.moveServoOut();
    } 
   /* else if (!left_bumper && !right_bumper ) {
      leftClimbPower = 0;
      rightClimbPower = 0;
    } */ else {
      leftClimbPower = 0;
      rightClimbPower = 0;
    }

    climbSubsystem.setLeftClimbPower(leftClimbPower);
    climbSubsystem.setRightClimbPower(rightClimbPower);

    if (driverController.getLeftTriggerAxis() > 0.25) {
      // up
      climbSubsystem.setLHighClimbPower(-1);
      climbSubsystem.setRHighClimbPower(1);
    } else if (driverController.getRightTriggerAxis() > 0.25) {
      // down
      climbSubsystem.setLHighClimbPower(1);
      climbSubsystem.setRHighClimbPower(-1);
    } else {
      climbSubsystem.setLHighClimbPower(0);
      climbSubsystem.setRHighClimbPower(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbSubsystem.stopClimb();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}