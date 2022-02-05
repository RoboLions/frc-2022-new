// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoMove;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultAutoPath extends SequentialCommandGroup {
  
  public DefaultAutoPath(final DriveSubsystem driveSubsystem) {
    super (new AutoMove(driveSubsystem, 1));
  }
}