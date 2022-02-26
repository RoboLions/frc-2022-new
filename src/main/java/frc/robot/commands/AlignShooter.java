// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AlignShooter extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  private final LimelightSubsystem limelightSubsystem;

  private final static XboxController driverController = RobotContainer.driverController;

  private final static double LIMELIGHT_SCALAR = 1;
  
  public double offsetX1 = 0;
  
  /** Creates a new AlignShooter. */
  public AlignShooter(LimelightSubsystem limelight, DriveSubsystem drive) {
    driveSubsystem = drive;
    limelightSubsystem = limelight;
    addRequirements(limelightSubsystem, driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    offsetX1 = LimelightSubsystem.getLimelightX() * LIMELIGHT_SCALAR; // continuously getting this 50 times/sec
    // System.out.println("offsetX: " + offsetX);

    /*
    if (driverController.getBButton()) {
      // offsetX = LimelightSubsystem.getLimelightX() * LIMELIGHT_SCALAR;
      double rotate = 0;    

      if (offsetX > 1) {
        rotate = 0.4;
      } else if (offsetX < -1) {
        rotate = -0.4;
      } else {
        rotate = 0;
      }

      driveSubsystem.driveWithRotation(0, rotate);
    }  
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}