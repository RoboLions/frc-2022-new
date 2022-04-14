package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoMoveDistance;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.AutoZeroYaw;
import frc.robot.commands.ResetDrivetrainEncoders;
import frc.robot.commands.StopNWait;
import frc.robot.subsystems.DriveSubsystem;

public class TestMove extends SequentialCommandGroup {

  public TestMove(final DriveSubsystem driveSubsystem) {
    super(

      new ParallelCommandGroup(
        //new AutoMoveArmDown(armSubsystem).withTimeout(1), // move arm down to pick up next ball
        new ResetDrivetrainEncoders(driveSubsystem).withTimeout(1), // reset drivetrain encoders to make auto path work correctly
        //new AutoTurnLLOn(limelightSubsystem).withTimeout(1),
        new AutoZeroYaw(driveSubsystem).withTimeout(1)
      ),

      new StopNWait(driveSubsystem, 0.1), // allow all past commands to settle out

      new AutoMoveDistance(driveSubsystem, 1), //move a distance of 0.7m

      new StopNWait(driveSubsystem, 0.5),

      new AutoMoveDistance(driveSubsystem, 1),

      new StopNWait(driveSubsystem, 0.5),

      new AutoTurn(driveSubsystem, 90),

      new StopNWait(driveSubsystem, 0.5),

      new AutoMoveDistance(driveSubsystem, 1),

      new StopNWait(driveSubsystem, 0.5),

      new AutoTurn(driveSubsystem, 90),

      new StopNWait(driveSubsystem, 0.5),

      new AutoMoveDistance(driveSubsystem, 1),

      new StopNWait(driveSubsystem, 0.5),

      new AutoMoveDistance(driveSubsystem, 1),

      new StopNWait(driveSubsystem, 0.5),

      new AutoTurn(driveSubsystem, 90),

      new StopNWait(driveSubsystem, 0.5),

      new AutoMoveDistance(driveSubsystem, 1),

      new StopNWait(driveSubsystem, 0.5),

      new AutoTurn(driveSubsystem, 90),

      new StopNWait(driveSubsystem, 0.5)
      // should be back at the start now
    );
  }
}