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
import frc.robot.commands.AutoMoveElevatorUp;
import frc.robot.commands.AutoMoveElevatorDown;
//import frc.robot.commands.AutoMoveAndIntake;
import frc.robot.commands.AutoShoot;
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

public class AutoPath7 extends SequentialCommandGroup {

  /** 
   * guardrail start (distance from center of ring to front label is _)
   * pre-loaded w/ 1 ball, shoot, move forward to intake ball, shoot
   * turn to terminal, move forward to intake ball at terminal, 
   * move within range of hub, shoot ball
  */

  public AutoPath7(final DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, 
  LimelightSubsystem limelightSubsystem, ShooterSubsystem shooterSubsystem, ArmSubsystem armSubsystem) {
    super(

      new ParallelCommandGroup(
        new AutoMoveArmDown(armSubsystem).withTimeout(1), 
        new ResetDrivetrainEncoders(driveSubsystem).withTimeout(1),
        new AutoTurnLLOn(limelightSubsystem).withTimeout(1)
      ),

      new AutoMove(driveSubsystem, -0.3),

      new AutoShoot(shooterSubsystem).withTimeout(2),

      // Shoot ball
      new AutoShootWithElevator(shooterSubsystem).withTimeout(1.5),

      new AutoTurn(driveSubsystem, -10),

      new ParallelCommandGroup(
        new AutoMove(driveSubsystem, -0.6),
        new AutoIntake(intakeSubsystem).withTimeout(2),
        new AutoShootWithElevator(shooterSubsystem).withTimeout(2),
        new AutoMoveArmDown(armSubsystem).withTimeout(2)
      ),

      new AutoMove(driveSubsystem, -0.4),

      new AutoTurn(driveSubsystem, 90),

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
        new AutoTurn(driveSubsystem, -15)
      ),

      new AutoShootWithElevator(shooterSubsystem).withTimeout(2)
    );
  }
}
