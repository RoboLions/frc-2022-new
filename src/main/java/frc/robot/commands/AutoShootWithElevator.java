// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SimpleShooterSubsystem;

public class AutoShootWithElevator extends CommandBase {
  /** Creates a new AutoShoot. */
  private final ShooterSubsystem shooterSubsystem;
  double hoodSpeed = 0;
  // private static final int DEFAULT_TIME = 1;

  public AutoShootWithElevator(ShooterSubsystem shooter) {
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
    LimelightSubsystem.setVisionProcessor();
    double x = LimelightSubsystem.getHorizontalDistance();
    double speed = 0.0581*x + 1.11;
    if (x >= 7 && x <= 11) {
      hoodSpeed = 0.4;
    } else if (x > 11 && x < 11.5) {
      hoodSpeed = 0.2*x - 1.8;
    } else if (x >= 11.5 && x <= 12) {
      hoodSpeed = 0.5;
    } else if (x > 12 && x < 13) {
      hoodSpeed = 0.2*x - 1.91;
    } else if (x >= 13 && x <= 13.5) {
      hoodSpeed = 0.7;
    } else if (x > 13.5 && x <= 14) {
      hoodSpeed = 0.2*x - 2;
    } else {
      hoodSpeed = 0;
    }

    shooterSubsystem.moveBeltUp();
    shooterSubsystem.steadyShoot(speed);
    shooterSubsystem.setHoodSpeed(hoodSpeed);
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
