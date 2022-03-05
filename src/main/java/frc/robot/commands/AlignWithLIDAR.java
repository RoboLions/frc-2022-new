// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LIDARLiteSubsystem;

public class AlignWithLIDAR extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  private final LIDARLiteSubsystem lidar;

  private final static XboxController driverController = RobotContainer.driverController;
  
  public AlignWithLIDAR(LIDARLiteSubsystem lidarLiteSubsystem, DriveSubsystem drive) {
    lidar = lidarLiteSubsystem;
    driveSubsystem = drive;
    addRequirements(lidar, driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // while distance > 0, rotate until distance = 0
    if (driverController.getAButton()) {
      if (lidar.getDistance() > 0.2) {
        driveSubsystem.driveWithRotation(0, 0.1);
      }
    } else {
      driveSubsystem.stop();
    }

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
}*/