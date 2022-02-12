// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class MoveClimb extends CommandBase {

  public static final double EXTEND_POWER = 0.5; 
  public static final double RETRACT_POWER = -1; 

  public static final double MAX_ENCODER_COUNT = 10; //TODO change value
  public static final double MIN_ENCODER_COUNT = 0;

  private final ClimbSubsystem climbSubsystem;
  private final XboxController driverController = RobotContainer.driverController;

  public static int climb_motion_state = 0;
  public static double climbEncoderCounts;
  
  /** Creates a new MoveClimb. */
  public MoveClimb(ClimbSubsystem climb) {
    // Use addRequirements() here to declare subsystem dependencies.
    climbSubsystem = climb;
    addRequirements(climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climbEncoderCounts = climbSubsystem.getEncoderPosition();
    double climbPower = 0;
    
    boolean left_bumper = driverController.getLeftBumper();
    boolean right_bumper = driverController.getRightBumper();

    if(climbEncoderCounts > MIN_ENCODER_COUNT) {
      climbPower = EXTEND_POWER; // moving inwards
  } 
  // Pull up climber
  else if(climbEncoderCounts < MAX_ENCODER_COUNT) {
      climbPower = RETRACT_POWER; // moving outwards
  } 
  
  else if(!left_bumper && !right_bumper ) {
      climbPower = 0; // not moving based on bumpers
  }

  climbSubsystem.setClimbPower(climbPower);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
