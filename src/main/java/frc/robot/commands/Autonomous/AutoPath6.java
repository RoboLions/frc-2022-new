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
import frc.robot.commands.AutoShootWithElevator;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.AutoTurnLLOn;
import frc.robot.commands.ResetDrivetrainEncoders;
import frc.robot.commands.StopNWait;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoPath6 extends SequentialCommandGroup {
  
   /** 
   * terminal start (distance from center of ring to front label is _)
   * pre-loaded w/ 1 ball, shoot, move forward to intake ball, shoot
   * turn to terminal, move forward to intake ball at terminal, 
   * move within range of hub, shoot ball
  */

  public AutoPath6(final DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, 
  ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem, ArmSubsystem armSubsystem){
    super(

      new ParallelCommandGroup(
        new AutoMoveArmDown(armSubsystem).withTimeout(1), 
        new ResetDrivetrainEncoders(driveSubsystem).withTimeout(1),
        new AutoTurnLLOn(limelightSubsystem).withTimeout(1)
      ),

      new AutoMove(driveSubsystem, -0.3),

      new AutoShootAt6(shooterSubsystem).withTimeout(1.5),

      // Shoot ball
      new AutoShoot6Elevator(shooterSubsystem).withTimeout(1.5),

      new AutoTurn(driveSubsystem, 5),

      new ParallelCommandGroup(
        new AutoMove(driveSubsystem, -0.6),
        new AutoIntake(intakeSubsystem).withTimeout(2),
        new AutoShoot6point9Elevator(shooterSubsystem).withTimeout(2),
        new AutoMoveArmDown(armSubsystem).withTimeout(2)
      ),

      new AutoTurn(driveSubsystem, 10),

      new AutoMove(driveSubsystem, -3),

      new ParallelCommandGroup(
        new AutoMove(driveSubsystem, -3.5),
        new AutoIntake(intakeSubsystem).withTimeout(3),
        new AutoMoveElevatorUp(shooterSubsystem).withTimeout(3),
        new AutoMoveArmDown(armSubsystem).withTimeout(3)
      ),

      new StopNWait(driveSubsystem, 0.5),

      new AutoMove(driveSubsystem, -2),

      new ParallelCommandGroup(
        new AutoMoveElevatorDown(shooterSubsystem).withTimeout(0.16),
        new AutoTurn(driveSubsystem, -25)
      ),

      new AutoShootWithElevator(shooterSubsystem).withTimeout(2)

    /*
      // intake facing away from hub, move forwards part of the way
      new AutoMove(driveSubsystem, 2),

      // intake ball with elevator going
      new ParallelRaceGroup(
        new AutoMove(driveSubsystem, 0.3),
        new AutoIntake(intakeSubsystem),
        new AutoMoveElevatorUp(shooterSubsystem)
      ),

      new StopNWait(driveSubsystem, 0.5),

       // Turn LL on to align and shoot
       new AutoTurnLLOn(limelightSubsystem),

      // Target hub
      //new AlignShooter(limelightSubsystem, driveSubsystem),

      new StopNWait(driveSubsystem, 0.5),

      // Shoot balls
      new AutoShoot(shooterSubsystem).withTimeout(4),

      // Turn to go to terminal
      new AutoTurn(driveSubsystem, 90),

      new StopNWait(driveSubsystem, 0.5),

      // Move forward to terminal but not all the way
      new AutoMove(driveSubsystem, 2.1),

      new StopNWait(driveSubsystem, 0.2),

      // Finish move forward to terminal with intake running
      new ParallelRaceGroup(
        new AutoMove(driveSubsystem, 0.1),
        new AutoIntake(intakeSubsystem),
        new AutoMoveElevatorUp(shooterSubsystem)
      ),

      new StopNWait(driveSubsystem, 0.5),

      // Move back to get in range of LL
      new AutoMove(driveSubsystem, -1),

      // Align to hub
      //new AlignShooter(limelightSubsystem, driveSubsystem),

      // Shoot
      new AutoShoot(shooterSubsystem).withTimeout(4)*/
    );
  }
}
 