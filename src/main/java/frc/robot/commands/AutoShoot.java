// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SimpleShooterSubsystem;

public class AutoShoot extends CommandBase {
  /** Creates a new AutoShoot. */
  private final ShooterSubsystem shooterSubsystem;
  // private static final int DEFAULT_TIME = 1;

  public AutoShoot(ShooterSubsystem shooter) {
    shooterSubsystem = shooter;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.stopBelt();
    shooterSubsystem.stopShooter();
    LimelightSubsystem.setVisionProcessor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LimelightSubsystem.setVisionProcessor();
    double speed = 1.74 + 0.0116 * (LimelightSubsystem.getHorizontalDistance() - 1.54) + 0.00684 * (LimelightSubsystem.getHorizontalDistance() - 1.54) * (LimelightSubsystem.getHorizontalDistance() - 1.54);

    shooterSubsystem.moveBeltUp();
    shooterSubsystem.steadyShoot(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopBelt();
    shooterSubsystem.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
