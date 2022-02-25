// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimbSubsystem;

public class MoveClimb extends CommandBase {

  public static final double UP_POWER = -0.4; 
  public static final double DOWN_POWER = 1; 

  public static final double SLOW_UP_POWER = -0.2;
  public static final double SLOW_DOWN_POWER = 0.2;

  public static final double TEST_UP_POWER = -0.1;
  public static final double TEST_DOWN_POWER = 0.1;

  public static final double R_MAX_ENCODER_COUNT = 330000;
  public static final double R_MIN_ENCODER_COUNT = 0;
  public static final double R_MID_TARGET_ENCODER_COUNT = 211000; // to climb high enough to the mid rung
  public static final double R_CLIMB_TARGET_ENCODER_COUNT = 16000; // to pull the robot up

  public static final double L_MAX_ENCODER_COUNT = 330000;
  public static final double L_MIN_ENCODER_COUNT = 0;
  public static final double L_MID_TARGET_ENCODER_COUNT = 211000; // to climb high enough to the mid rung
  public static final double L_CLIMB_TARGET_ENCODER_COUNT = 16000; // to pull the robot up

  private final ClimbSubsystem climbSubsystem;
  private final XboxController driverController = RobotContainer.driverController;

  public static int climb_motion_state = 0;

  public static double climbPower = 0;
  public static double rightStartingPosition;
  public static double leftStartingPosition;

  /** Creates a new MoveClimb. */
  public MoveClimb(ClimbSubsystem climb) {
    // Use addRequirements() here to declare subsystem dependencies.
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
    
    boolean left_bumper = driverController.getLeftBumper();
    boolean right_bumper = driverController.getRightBumper();

    boolean start_button = driverController.getStartButton();
    boolean back_button = driverController.getBackButton();
    
    System.out.println("Right Encoder Position:" + rightCurrentPosition);
    System.out.println("Left Encoder Position:" + leftCurrentPosition);

    if ((climbSubsystem.getRightLimitSwitchValue() == 1) || (climbSubsystem.getLeftLimitSwitchValue() == 1)) {
      climbSubsystem.resetEncoder();
    }

    // Pull down climber DURING COMPETITION (climbing)
    if (left_bumper && 
       (rightCurrentPosition > R_CLIMB_TARGET_ENCODER_COUNT) && 
       (leftCurrentPosition > L_CLIMB_TARGET_ENCODER_COUNT)) {
      climbPower = DOWN_POWER; // moving inwards
    }
    // Pull up climber to target position DURING COMPETITION
    else if (right_bumper && 
            (rightCurrentPosition < R_MID_TARGET_ENCODER_COUNT) &&
            (leftCurrentPosition < L_MID_TARGET_ENCODER_COUNT)) {
      climbPower = UP_POWER;
    } 
    // Pull down climber to reset (home)
    else if (back_button && 
            (rightCurrentPosition > R_MIN_ENCODER_COUNT) &&
            (leftCurrentPosition > L_MIN_ENCODER_COUNT)) {
      climbPower = SLOW_DOWN_POWER;
    }
    // Pull up climber to max position
    else if (start_button && 
            (rightCurrentPosition < R_MAX_ENCODER_COUNT) &&
            (leftCurrentPosition < L_MAX_ENCODER_COUNT)) {
      climbPower = UP_POWER; // moving outwards
    } 
    else if (!left_bumper && !right_bumper ) {
      climbPower = 0; // not moving based on bumpers
    } else {
      climbPower = 0;
    }

    /*
    if (left_bumper) {
      climbPower = EXTEND_POWER;
    } else if (right_bumper) {
      climbPower = RETRACT_POWER;
    }*/

    climbSubsystem.setClimbPower(climbPower);
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