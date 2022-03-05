// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignShooter;
//import frc.robot.commands.AlignWithLIDAR;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoMove;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.StopNWait;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
//import frc.robot.subsystems.LIDARLiteSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SimpleShooterSubsystem;

public class AutoPath2 extends SequentialCommandGroup {

  /** 
   * path that picks up a ball off the field then shoots both balls 
  */

  public AutoPath2(final DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, 
  LimelightSubsystem limelightSubsystem, ShooterSubsystem shooterSubsystem) {
    super(
      // Target ball on field
      //new AlignWithLIDAR(lidar, driveSubsystem), 
      // new AutoTurn(driveSubsystem, 90),

      //new StopNWait(driveSubsystem, 0.5),
      
      // Drive to ball on field
      new AutoMove(driveSubsystem, 0.5), // convert to meters

      new StopNWait(driveSubsystem, 0.5),
      
      // a crumb of intake command pls
      new AutoIntake(intakeSubsystem).withTimeout(2),

      new StopNWait(driveSubsystem, 0.5),

      // Target hub
      new AlignShooter(limelightSubsystem, driveSubsystem),

      new StopNWait(driveSubsystem, 0.5),

      // Shoot balls
      new AutoShoot(shooterSubsystem).withTimeout(6)
    );
  }
}