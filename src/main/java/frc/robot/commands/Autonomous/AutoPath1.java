// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.AutoMove;
import frc.robot.commands.AutoMoveArmDown;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutoShootWithElevator;
import frc.robot.commands.AutoTurnLLOn;
import frc.robot.commands.ResetDrivetrainEncoders;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoPath1 extends SequentialCommandGroup {

  /**
   * pre-loaded with 1 ball, drive off tarmac, shoot ball
   * works for any position
  */

  public AutoPath1(final DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, 
  LimelightSubsystem limelightSubsystem, ShooterSubsystem shooterSubsystem) {
    super(
      new ParallelCommandGroup(
        new ResetDrivetrainEncoders(driveSubsystem).withTimeout(1),
        new AutoTurnLLOn(limelightSubsystem).withTimeout(1)
      ),

      new AutoMove(driveSubsystem, -1.3),

      new AutoShoot(shooterSubsystem).withTimeout(1),

      // Shoot ball
      new AutoShootWithElevator(shooterSubsystem).withTimeout(1.5)
    );
  }
}