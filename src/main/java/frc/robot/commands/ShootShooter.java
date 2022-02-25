// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootShooter extends CommandBase {

  private final static XboxController manipulatorController = RobotContainer.manipulatorController;
  private final ShooterSubsystem shooterSubsystem;
  
  public ShootShooter(ShooterSubsystem shooter) {
    shooterSubsystem = shooter;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.stopShooter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // function: 0.09x+1.7
    // after adding in P value:
    // 0.75 good for low goal
    // 1.8 good for 1 feet
    // 2 good for 3 feet
    // 2.3 good for 6 feet
    // 2.5 good for 9 feet
    // 2.65 good for 11 feet

    if (manipulatorController.getXButton()) {
      shooterSubsystem.moveBeltUp();
      shooterSubsystem.steadyShoot(1);
    } else if (manipulatorController.getBButton()) {
      shooterSubsystem.moveBeltDown();
    } else {
      shooterSubsystem.stopBelt();
    }
  
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