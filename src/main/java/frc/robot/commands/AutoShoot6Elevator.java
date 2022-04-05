// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShoot6Elevator extends CommandBase {
  /** Creates a new AutoShoot6Elevator. */
  private final ShooterSubsystem shooterSubsystem;
  double hoodSpeed = 0;
  
  public AutoShoot6Elevator(ShooterSubsystem shooter) {
    shooterSubsystem = shooter;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.stopShooter();
    LimelightSubsystem.setVisionProcessor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.steadyShoot(1.77);
    shooterSubsystem.moveBeltUp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopShooter();
    shooterSubsystem.stopBelt();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
