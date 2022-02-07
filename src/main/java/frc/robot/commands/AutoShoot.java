// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SimpleShooterSubsystem;

public class AutoShoot extends CommandBase {
  /** Creates a new AutoShoot. */
  public AutoShoot(boolean upper) {
    // Use addRequirements() here to declare subsystem dependencies.
    if(upper == true) {
      SimpleShooterSubsystem.moveBeltUp();
      SimpleShooterSubsystem.shootUpperHub();
      SimpleShooterSubsystem.stopShooter();
    }
    else if(upper == false) {
      SimpleShooterSubsystem.moveBeltUp();
      SimpleShooterSubsystem.shootLowerHub();
      SimpleShooterSubsystem.stopShooter();
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
