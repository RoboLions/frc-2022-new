// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignShooter;
//import frc.robot.commands.AlignWithLIDAR;
//import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoMove;
//import frc.robot.commands.AutoMoveAndIntake;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.StopNWait;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
//import frc.robot.subsystems.LIDARLiteSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SimpleShooterSubsystem;

public class AutoPath4 extends SequentialCommandGroup {
  
  /** 
   * start from position second farthest from the hanger zone, but same tarmac as path 3
   * pre-loaded with 1 ball, move forward to intake ball, shoot both balls,
   * turn to terminal, move forward to intake ball at terminal, 
   * move within range of hub for LL, shoot ball
  */

  public AutoPath4(final DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, 
  ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem){
    super(

      // intake facing away from hub, move forwards and intake ball
      //new AutoMoveAndIntake(driveSubsystem, intakeSubsystem, 0.5),

      new StopNWait(driveSubsystem, 0.5),

      // Target hub
      new AlignShooter(limelightSubsystem, driveSubsystem),

      new StopNWait(driveSubsystem, 0.5),

      // Shoot balls
      new AutoShoot(shooterSubsystem).withTimeout(6),

      new AutoTurn(driveSubsystem, 10),

      new StopNWait(driveSubsystem, 0.5),

      // Move forward to terminal with intake running
      //new AutoMoveAndIntake(driveSubsystem, intakeSubsystem, 2.7),

      new StopNWait(driveSubsystem, 0.5),

      // Align to hub
      new AlignShooter(limelightSubsystem, driveSubsystem),

      // Shoot
      new AutoShoot(shooterSubsystem).withTimeout(6)
    );
  }
}
 