// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.AlignShooter;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoMove;
import frc.robot.commands.AutoMoveArmDown;
import frc.robot.commands.AutoMoveElevatorUp;
import frc.robot.commands.AutoMoveElevatorDown;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutoShoot6Elevator;
import frc.robot.commands.AutoShoot6point9Elevator;
import frc.robot.commands.AutoShootAt6;
import frc.robot.commands.AutoShootShort;
import frc.robot.commands.AutoShootWithElevator;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.AutoTurnLLOn;
import frc.robot.commands.AutoZeroYaw;
import frc.robot.commands.ResetDrivetrainEncoders;
import frc.robot.commands.StopNWait;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoPath8 extends SequentialCommandGroup {

  /** 
   * improved for states comp
   * pre-loaded with 1 ball, pick 1 ball off field, shoot both balls, move back slightly
   * bot at hangar start (distance from center of ring to front label is 50 inches)
  */

  public AutoPath8(final DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, 
  LimelightSubsystem limelightSubsystem, ShooterSubsystem shooterSubsystem, ArmSubsystem armSubsystem) {
    super(
      new ParallelCommandGroup(
        new AutoMoveArmDown(armSubsystem).withTimeout(1), 
        new ResetDrivetrainEncoders(driveSubsystem).withTimeout(1),
        new AutoTurnLLOn(limelightSubsystem).withTimeout(1)
      ),

      new AutoMove(driveSubsystem, -0.7),//move 0.7m

      new StopNWait(driveSubsystem, 0.5),

      new AutoShoot(shooterSubsystem).withTimeout(1),

      // Shoot ball
      new ParallelCommandGroup(
        new AutoShootWithElevator(shooterSubsystem).withTimeout(1.5),
        new AutoMoveArmDown(armSubsystem).withTimeout(1.5)
      ),
      
      new ParallelCommandGroup(
        new AutoMove(driveSubsystem, -0.26), //move another 0.26 m
        new AutoIntake(intakeSubsystem).withTimeout(1.5),
        new AutoShootWithElevator(shooterSubsystem).withTimeout(2.8),
        new AutoMoveArmDown(armSubsystem).withTimeout(2)
      ),

      new StopNWait(driveSubsystem, 0.25),

      new AutoMove(driveSubsystem, -0.7), //move another 0.5m

      new StopNWait(driveSubsystem, 0.25)

      /*new StopNWait(driveSubsystem, 0.2),

      new AutoZeroYaw(driveSubsystem).withTimeout(0.2),

      new StopNWait(driveSubsystem, 0.1),

      new AutoTurn(driveSubsystem, -90)*/

      /*new StopNWait(driveSubsystem, 0.25),

      new ParallelCommandGroup(
        new AutoMove(driveSubsystem, -0.9),
        new AutoIntake(intakeSubsystem).withTimeout(2),
        new AutoShootShort(shooterSubsystem).withTimeout(2.5),
        new AutoMoveArmDown(armSubsystem).withTimeout(2)
      )*/
    );

    /*
      new ParallelCommandGroup(
        new AutoMoveArmDown(armSubsystem).withTimeout(1), 
        new ResetDrivetrainEncoders(driveSubsystem).withTimeout(1),
        new AutoTurnLLOn(limelightSubsystem).withTimeout(1)
      ),

      new AutoMove(driveSubsystem, -0.35),

      new StopNWait(driveSubsystem, 0.5),

      new AutoShoot(shooterSubsystem).withTimeout(1.5),

      // Shoot ball
      new ParallelCommandGroup(
        new AutoShootWithElevator(shooterSubsystem).withTimeout(1.5),
        new AutoMoveArmDown(armSubsystem).withTimeout(1.5)
      ),
      
      new ParallelCommandGroup(
        new AutoMove(driveSubsystem, -0.75),
        new AutoIntake(intakeSubsystem).withTimeout(2),
        new AutoShootWithElevator(shooterSubsystem).withTimeout(2),
        new AutoMoveArmDown(armSubsystem).withTimeout(2)
      ),

      new StopNWait(driveSubsystem, 0.25),

      new AutoMove(driveSubsystem, -0.8),

      new StopNWait(driveSubsystem, 0.5)
    );*/
  }
}