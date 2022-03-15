// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoMoveAndIntake extends CommandBase {
  private final DriveSubsystem drivesubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private double start_dist_meters;
  private double target_distance;

  public AutoMoveAndIntake(final DriveSubsystem subsystem, final IntakeSubsystem intake, double distance) {
    drivesubsystem = subsystem;
    intakeSubsystem = intake;
    addRequirements(drivesubsystem, intakeSubsystem);
    start_dist_meters = drivesubsystem.distanceTravelledinMeters();
    target_distance = distance;
  }

  @Override
  public void initialize() {
    drivesubsystem.resetEncoders();
    drivesubsystem.ZeroYaw();
    start_dist_meters = drivesubsystem.distanceTravelledinMeters();
    drivesubsystem.state_flag_motion_profile = true;
  }

  @Override
  public void execute() { 
    // This function is constantly being called in the class at 50 Hz
    // This implements a straight move without any heading control 
    //double position_profile_command = drivesubsystem.positionMotionProfile.execute();
    //double feed_forward_rate = drivesubsystem.positionMotionProfile.velocity_feed_forward;                
    //System.out.println("Command: " + position_profile_command + " , FF: " + feed_forward_rate);
    drivesubsystem.autoDrive(target_distance, 0.0);
    intakeSubsystem.intakeBalls();
    // System.out.println("AUTO WORKS");
  }

  @Override
  public boolean isFinished() {
    // This function is constantly being called in the class at 50 Hz
    // This helps to determine when you are done with the command
    //boolean tempReturn = false;
    double distance_driven = drivesubsystem.distanceTravelledinMeters() - start_dist_meters;
    double positionError = Math.abs(target_distance - distance_driven);
    return(positionError < 0.01); // stop whenever we go the commanded distance within 1 cm
    //return(tempReturn);
  } 
}	