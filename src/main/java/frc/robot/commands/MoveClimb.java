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

  public static final double MAX_ENCODER_COUNT = 330000;
  public static final double MIN_ENCODER_COUNT = 0;
  public static final double MID_TARGET_ENCODER_COUNT = 211000;
  public static final double CLIMB_TARGET_ENCODER_COUNT = 70000;

  private final ClimbSubsystem climbSubsystem;
  private final XboxController driverController = RobotContainer.driverController;

  public static int climb_motion_state = 0;

  public static double climbPower = 0;
  public static double startingPosition;

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
    startingPosition = Math.abs(climbSubsystem.getEncoderPosition());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPosition = Math.abs(climbSubsystem.getEncoderPosition());
    
    boolean left_bumper = driverController.getLeftBumper();
    boolean right_bumper = driverController.getRightBumper();

    boolean start_button = driverController.getStartButton();
    boolean back_button = driverController.getBackButton();

    double targetPosition = startingPosition + 10000.0;
    //currentPosition = Math.abs(currentPosition);
    
    System.out.println("Encoder starting Position:" + startingPosition);
    System.out.println("Encoder target Position:" + targetPosition);
    System.out.println("Encoder Position:" + currentPosition);

    if (climbSubsystem.getLimitSwitchValue() == 1) {
      climbSubsystem.resetEncoder();
    }
/*
    if (left_bumper) {
      climbPower = 0.1;
    } else if (right_bumper) {
      climbPower = -0.1;
    } else if (!left_bumper && !right_bumper) {
      climbPower = 0;
    }*/

    /*if (climbSubsystem.getLimitSwitchValue() == 0) { // if open
      climbPower = 0.1;
    } else {
      climbPower = 0;
    }
*/
    /*
    if (left_bumper && 
       (currentPosition <= targetPosition)) {
         // if encoder counts right now <= starting encoder counts + 10,000 counts, keep going down
      climbPower = 0.1;
    } else {
      climbPower = 0;
    }
    
    if (right_bumper) {//&& ((Math.abs(climbEncoderCounts - climbEncoderCountsNow)) <= 2)) {
      climbPower = -0.1;
    } else {
      climbPower = 0;
    }
    
    if (!left_bumper && !right_bumper) {
      climbPower = 0;
    }*/

    // Pull down climber DURING COMPETITION (climbing)
    if (left_bumper && (currentPosition > CLIMB_TARGET_ENCODER_COUNT)) {
      climbPower = DOWN_POWER; // moving inwards
    }
    // Pull up climber to target position DURING COMPETITION
    else if (right_bumper && (currentPosition < MID_TARGET_ENCODER_COUNT)) {
      climbPower = UP_POWER;
    } 
    // Pull down climber to reset (home)
    else if (back_button && (currentPosition > MIN_ENCODER_COUNT)) {
      climbPower = SLOW_DOWN_POWER;
    }
    // Pull up climber to max position
    else if (start_button && (currentPosition < MAX_ENCODER_COUNT)) {
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