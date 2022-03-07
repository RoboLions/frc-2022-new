// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArm extends CommandBase {
  
  private final ArmSubsystem armSubsystem;
  private final XboxController manipulatorController = RobotContainer.manipulatorController;

  public static double armStartingPosition;

  public MoveArm(ArmSubsystem arm) {
    armSubsystem = arm;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armStartingPosition = armSubsystem.getPitch();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (manipulatorController.getYButton()) {
      if (armStartingPosition < 75) {
        armSubsystem.setArmPower(0.5);
      }
      else {
        armSubsystem.setArmPower(0);
      }
    } else if (manipulatorController.getBButton()) {
      if (armStartingPosition > -21) {
        armSubsystem.setArmPower(-0.7);
      } else {
        armSubsystem.setArmPower(0);
      }
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
