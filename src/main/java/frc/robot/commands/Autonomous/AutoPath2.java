// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.AlignShooter;
import frc.robot.commands.AutoIntake;
//import frc.robot.commands.AlignWithLIDAR;
//import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoMove;
import frc.robot.commands.AutoMoveArmDown;
import frc.robot.commands.AutoMoveElevator;
import frc.robot.commands.AutoMoveElevatorDown;
//import frc.robot.commands.AutoMoveAndIntake;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutoShootShort;
import frc.robot.commands.AutoShootWithElevator;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.AutoTurnLLOn;
import frc.robot.commands.ResetDrivetrainEncoders;
import frc.robot.commands.StopNWait;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
//import frc.robot.subsystems.LIDARLiteSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SimpleShooterSubsystem;

public class AutoPath2 extends SequentialCommandGroup {

  /** 
   * pre-loaded with 1 ball, pick 1 ball off field, shoot both balls
   * bot must be directly in front of a ball
  */

  public AutoPath2(final DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, 
  LimelightSubsystem limelightSubsystem, ShooterSubsystem shooterSubsystem, ArmSubsystem armSubsystem) {
    super(

    // ball 40.44 in from end of tarmac tape
    // 7.58 in from center of ball to tarmac corner straight out 
    // tarmac tape 75.07 in

      new AutoMoveArmDown(armSubsystem).withTimeout(1), 

      new ResetDrivetrainEncoders(driveSubsystem).withTimeout(1),

      new AutoTurnLLOn(limelightSubsystem).withTimeout(0.5),

      new AutoMove(driveSubsystem, -0.3),

      new AutoShoot(shooterSubsystem).withTimeout(2.5),

      // Shoot balls
      new AutoShootWithElevator(shooterSubsystem).withTimeout(1.5),

      new ParallelCommandGroup(
        new AutoMove(driveSubsystem, -0.5),
        new AutoIntake(intakeSubsystem).withTimeout(2),
        new AutoShoot(shooterSubsystem).withTimeout(2),
        new AutoMoveArmDown(armSubsystem).withTimeout(2)
      ),

      new ParallelRaceGroup(
        new AutoMove(driveSubsystem, -0.8),
        new AutoIntake(intakeSubsystem).withTimeout(4),
        new AutoMoveArmDown(armSubsystem),
        new AutoShootWithElevator(shooterSubsystem).withTimeout(4)
      )

      //new StopNWait(driveSubsystem, 1),

      //new AutoMove(driveSubsystem, -1.5)

      /*
      new AutoMove(driveSubsystem, -0.6),

      new AutoShoot(shooterSubsystem).withTimeout(1),

      new AutoShootWithElevator(shooterSubsystem).withTimeout(1.5),

      new AutoMove(driveSubsystem, -1.5)*/
    );
  }
}