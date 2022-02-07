// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DriveSubsystem;

public class StopNWait extends CommandBase {
  protected Timer m_Timer = new Timer(); 
  private final double m_duration;
  private final DriveSubsystem driveSubsystem; 
  /** Creates a new StopNWait. */
  public StopNWait(DriveSubsystem drivesubsystem, double seconds ) {
    // Use addRequirements() here to declare subsystem dependencies. fgdfg
m_duration = seconds;
SendableRegistry.setName(this, getName() + ":" + seconds + " seconds");
driveSubsystem = drivesubsystem;
//addRequirements(driveSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.stop();
    m_Timer.reset();
    m_Timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Timer.advanceIfElapsed(m_duration);
   
  }
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
