// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoMove;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutoTurn;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestPath extends SequentialCommandGroup {
  /** Creates a new TestPath. */
  public TestPath(final DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem) {
    super (
      /*new AutoMove(driveSubsystem, -0.9),
      new AutoMove(driveSubsystem, -0.5)*/
      //new AutoTurn(driveSubsystem, 90),
      //new AutoMove(driveSubsystem, -0.5)
      
      new ParallelCommandGroup(
        new AutoMove(driveSubsystem, -0.3),
        new AutoShoot(shooterSubsystem).withTimeout(1)
      )

      /*

      new ParallelCommandGroup(
        new AutoMove(driveSubsystem, -1),
        new AutoIntake(intakeSubsystem).withTimeout(2),
        new AutoMoveElevator(shooterSubsystem).withTimeout(2),
        new AutoMoveArmDown(armSubsystem).withTimeout(2)
      ),

      new AutoMoveElevatorDown(shooterSubsystem).withTimeout(0.2),
      */
    );
  }
}
