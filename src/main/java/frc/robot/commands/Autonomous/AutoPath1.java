// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignShooter;
// import frc.robot.commands.AlignWithLIDAR;
import frc.robot.commands.AutoMove;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutoTurnLLOn;
// import frc.robot.commands.ShootShooter;
import frc.robot.commands.StopNWait;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SimpleShooterSubsystem;

public class AutoPath1 extends SequentialCommandGroup {

  /**
   * pre-loaded with 1 ball, drive off tarmac, shoot ball
   * works for any position
  */

  public AutoPath1(final DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, 
  LimelightSubsystem limelightSubsystem, ShooterSubsystem shooterSubsystem) {
    super(
      // intake facing away from hub, move forwards
      new AutoMove(driveSubsystem, 2.7), // TODO: correct distance

      new StopNWait(driveSubsystem, 0.5),

      // Turn LL on to align and shoot
      new AutoTurnLLOn(limelightSubsystem),

      // Target hub
      new AlignShooter(limelightSubsystem, driveSubsystem),

      new StopNWait(driveSubsystem, 0.5),

      // Shoot balls
      new AutoShoot(shooterSubsystem).withTimeout(6)
    );
  }
}