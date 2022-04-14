// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoMoveDistance;
import frc.robot.commands.AutoMoveArmDown;
import frc.robot.commands.AutoMoveElevatorUp;
import frc.robot.commands.AutoReverseShoot;
import frc.robot.commands.AutoMoveElevatorDown;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutoShoot6Elevator;
import frc.robot.commands.AutoShootAt6;
import frc.robot.commands.AutoShootFar;
import frc.robot.commands.AutoShootFarNoE;
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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestPath extends SequentialCommandGroup {
  /** Creates a new TestPath. */
  public TestPath(final DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, 
  ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem, ArmSubsystem armSubsystem) {
    super (
      
      new ParallelCommandGroup(
        new AutoMoveArmDown(armSubsystem).withTimeout(0.5), 
        new ResetDrivetrainEncoders(driveSubsystem).withTimeout(0.5),
        new AutoTurnLLOn(limelightSubsystem).withTimeout(0.5)
      ),

      new ParallelCommandGroup(
        new AutoMoveDistance(driveSubsystem, -4.8),
        new AutoIntake(intakeSubsystem).withTimeout(5),
        new AutoMoveElevatorUp(shooterSubsystem).withTimeout(5),
        new AutoMoveArmDown(armSubsystem).withTimeout(5)
      ),

      new StopNWait(driveSubsystem, 0.5),

      new ParallelCommandGroup(
        new AutoReverseShoot(shooterSubsystem).withTimeout(0.18),
        new AutoZeroYaw(driveSubsystem).withTimeout(0.5)
      ),

      new StopNWait(driveSubsystem, 0.25),
      
      new AutoTurn(driveSubsystem, 10),

      new StopNWait(driveSubsystem, 0.25),

      new AutoShootFarNoE(shooterSubsystem).withTimeout(1),

      // Shoot ball
      new ParallelCommandGroup(
        new AutoShootFar(shooterSubsystem).withTimeout(0.15),
        new AutoMoveArmDown(armSubsystem).withTimeout(0.15)
      ),

      new AutoShootFarNoE(shooterSubsystem).withTimeout(0.5),

      new ParallelCommandGroup(
        new AutoIntake(intakeSubsystem).withTimeout(5),
        new AutoShootFar(shooterSubsystem).withTimeout(5),
        new AutoMoveArmDown(armSubsystem).withTimeout(5)
      )
      /*
      new ParallelCommandGroup(
        new AutoMoveArmDown(armSubsystem).withTimeout(0.5), 
        new ResetDrivetrainEncoders(driveSubsystem).withTimeout(0.5),
        new AutoTurnLLOn(limelightSubsystem).withTimeout(0.5)
      ),

      new ParallelCommandGroup(
        new AutoMove(driveSubsystem, -1.7),
        new AutoIntake(intakeSubsystem).withTimeout(2.15),
        new AutoMoveElevatorUp(shooterSubsystem).withTimeout(2.15),
        new AutoMoveArmDown(armSubsystem).withTimeout(2.15)
      ),

      new ParallelCommandGroup(
        new StopNWait(driveSubsystem, 0.5),
        new AutoZeroYaw(driveSubsystem).withTimeout(0.5)
      ),
      
      new AutoReverseShoot(shooterSubsystem).withTimeout(0.13),

      new AutoTurn(driveSubsystem, 9),

      new AutoShoot(shooterSubsystem).withTimeout(1),

      // Shoot ball
      new ParallelCommandGroup(
        new AutoShootWithElevator(shooterSubsystem).withTimeout(0.15),
        new AutoMoveArmDown(armSubsystem).withTimeout(0.15)
      ),

      new AutoShoot(shooterSubsystem).withTimeout(0.5),

      new ParallelCommandGroup(
        new AutoShootWithElevator(shooterSubsystem).withTimeout(1),
        new AutoMoveArmDown(armSubsystem).withTimeout(1),
        new AutoZeroYaw(driveSubsystem).withTimeout(1)
      ),

      new AutoTurn(driveSubsystem, -33),

      new StopNWait(driveSubsystem, 0.5),

      new ParallelCommandGroup(
        new AutoMove(driveSubsystem, -3.5),
        new AutoIntake(intakeSubsystem).withTimeout(6),
        new AutoMoveArmDown(armSubsystem).withTimeout(6),
        new AutoMoveElevatorUp(shooterSubsystem).withTimeout(6)
      ),

      new ParallelCommandGroup(
        new StopNWait(driveSubsystem, 0.7),
        new AutoZeroYaw(driveSubsystem).withTimeout(0.7)
      ),
      
      new AutoTurn(driveSubsystem, 10),

      new StopNWait(driveSubsystem, 0.5),

      new AutoReverseShoot(shooterSubsystem).withTimeout(0.25),

      new StopNWait(driveSubsystem, 0.15),

      new AutoShootFarNoE(shooterSubsystem).withTimeout(1),

      new AutoShootFar(shooterSubsystem).withTimeout(0.15),

      new AutoShootFarNoE(shooterSubsystem).withTimeout(0.5),

      new AutoShootFar(shooterSubsystem).withTimeout(1)
      */

      /*

      new StopNWait(driveSubsystem, 0.5),

      new AutoMoveElevatorDown(shooterSubsystem).withTimeout(0.28),

      new AutoMove(driveSubsystem, -2.5),

      new StopNWait(driveSubsystem, 0.5),

      new AutoShootWithElevator(shooterSubsystem).withTimeout(1)*/
    );
  }
}
