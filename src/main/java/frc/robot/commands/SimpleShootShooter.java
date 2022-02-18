// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SimpleShooterSubsystem;

public class SimpleShootShooter extends CommandBase {

  private final static XboxController manipulatorController = RobotContainer.manipulatorController;
  private final SimpleShooterSubsystem shooterSubsystem;
  
  public SimpleShootShooter(SimpleShooterSubsystem shooter) {
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

    if (manipulatorController.getAButton()) {
      //shooterSubsystem.moveBeltUp();
      shooterSubsystem.shootUpperHub();
    } else if (manipulatorController.getBButtonPressed()) {
      //shooterSubsystem.moveBeltUp();
      shooterSubsystem.shootLowerHub();
    } else {
      //shooterSubsystem.stopBelt();
      shooterSubsystem.stopShooter();
    }
  
    /*
    // testing Limelight
    if (manipulatorController.getYButtonPressed()) {
      shooterSubsystem.autoShootUpper();
      shooterSubsystem.moveBeltUp();
    } else {
      shooterSubsystem.stopBelt();
      shooterSubsystem.stopShooter();
    }*/
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
