// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.AlignShooter;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoMoveDistance;
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

public class AutoPath9 extends SequentialCommandGroup {

  /** 
   * improved for states comp
   * pre-loaded with 1 ball, pick 1 ball off field, shoot both balls, rude auto
   * bot at hangar start (distance from center of ring to front label is 50 inches)
  */

  public AutoPath9(final DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, 
  LimelightSubsystem limelightSubsystem, ShooterSubsystem shooterSubsystem, ArmSubsystem armSubsystem) {
    super(
      new ParallelCommandGroup(
        new AutoMoveArmDown(armSubsystem).withTimeout(1), // move arm down to pick up next ball
        new ResetDrivetrainEncoders(driveSubsystem).withTimeout(1), // reset drivetrain encoders to make auto path work correctly
        new AutoTurnLLOn(limelightSubsystem).withTimeout(1)
        //new AutoZeroYaw(driveSubsystem).withTimeout(1)
      ),

      new StopNWait(driveSubsystem, 1), // allow all past commands to settle out

      new AutoMoveDistance(driveSubsystem, -0.7), //move a distance of 0.7m

      //new StopNWait(driveSubsystem, 0.5), 

      new AutoShoot(shooterSubsystem).withTimeout(1),

      // Shoot ball
      new ParallelCommandGroup(
        new AutoShootWithElevator(shooterSubsystem).withTimeout(1.5),
        new AutoMoveArmDown(armSubsystem).withTimeout(1.5)
      ),
      
      new ParallelCommandGroup(
        new AutoMoveDistance(driveSubsystem, -0.26), //move another 0.26 m
        new AutoIntake(intakeSubsystem).withTimeout(1.5),
        new AutoShootWithElevator(shooterSubsystem).withTimeout(2.8),
        new AutoMoveArmDown(armSubsystem).withTimeout(2)
      ),

      new StopNWait(driveSubsystem, 0.25),

      //new AutoZeroYaw(driveSubsystem).withTimeout(0.2),

      //new StopNWait(driveSubsystem, 0.1),

      new AutoTurn(driveSubsystem, -89), //-89

      //new StopNWait(driveSubsystem, 0.1),

      //new AutoZeroYaw(driveSubsystem).withTimeout(0.2),

      //new StopNWait(driveSubsystem, 0.1),

      new ParallelCommandGroup(
        new AutoMoveDistance(driveSubsystem, -0.6), //move another 0.2m
        new AutoIntake(intakeSubsystem).withTimeout(2),
        new AutoShootShort(shooterSubsystem).withTimeout(3),
        new AutoMoveArmDown(armSubsystem).withTimeout(2)
      )
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