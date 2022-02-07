// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
// import edu.wpi.first.wpilibj.XboxController;
// import frc.robot.commands.AutoMove.Mode;

public class AutoIntake extends CommandBase {
  /** Creates a new Intake. */
  private final IntakeSubsystem intakeSubsystem;
  // private static final int DEFAULT_TIME = 1;

  public AutoIntake(IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    intakeSubsystem = intake;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.intakeBalls();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}