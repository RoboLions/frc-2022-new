// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoZeroYaw extends CommandBase {
  
  private final DriveSubsystem drivesubsystem;
  private boolean endme = false;

  public AutoZeroYaw(final DriveSubsystem subsystem) {
    drivesubsystem = subsystem;
    addRequirements(drivesubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivesubsystem.ZeroYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (drivesubsystem.getYaw() == 0) {
      endme = true;
    } else {
      endme = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endme = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endme;
  }
}
