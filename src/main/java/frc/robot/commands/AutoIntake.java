// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import frc.robot.commands.MotorIntake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoIntake extends CommandBase {
  /** Creates a new AutoIntake. */

  //remember true is take in false is take out
  public AutoIntake(boolean OutOrIn) {
    // Use addRequirements() here to declare subsystem dependencies.

    if(OutOrIn == true) {
      IntakeSubsystem.intakeBalls();
    }
    else if(OutOrIn == false) {
      IntakeSubsystem.outtakeBalls();
    }


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
