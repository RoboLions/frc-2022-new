// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimbSubsystem;

public class MoveClimb extends CommandBase {

  public static final double EXTEND_POWER = 0.1; 
  public static final double RETRACT_POWER = -0.1; 

  /*
  public static final double MAX_ENCODER_COUNT = 296461;
  public static final double MIN_ENCODER_COUNT = 380000;*/

  private final ClimbSubsystem climbSubsystem;
  private final XboxController driverController = RobotContainer.driverController;

  public static int climb_motion_state = 0;

  public static double climbPower = 0;
  public static double climbEncoderCounts;
  public static double climbEncoderCountsNow;

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

    climbEncoderCounts = climbSubsystem.getEncoderPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climbEncoderCountsNow = climbSubsystem.getEncoderPosition();
    
    boolean left_bumper = driverController.getLeftBumper();
    boolean right_bumper = driverController.getRightBumper();

    System.out.println("Encoder Position:" + climbEncoderCountsNow);

    if (left_bumper && ((climbEncoderCounts + 50000) > climbEncoderCountsNow )) {
      climbPower = 0.1;
    } else if (right_bumper && ((climbEncoderCounts - 50000) < climbEncoderCountsNow)) {
      climbPower = -0.1;
    } else if (!left_bumper && !right_bumper) {
      climbPower = 0;
    }

    /*
    if (left_bumper && (climbEncoderCounts > MIN_ENCODER_COUNT)) {
      climbPower = EXTEND_POWER; // moving inwards
    } 
    // Pull up climber
    else if (right_bumper && (climbEncoderCounts < MAX_ENCODER_COUNT)) {
      climbPower = RETRACT_POWER; // moving outwards
    } 
    else if (!left_bumper && !right_bumper ) {
      climbPower = 0; // not moving based on bumpers
    }*/

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