package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoMove;
import frc.robot.commands.ResetDrivetrainEncoders;
import frc.robot.subsystems.DriveSubsystem;

public class TestMove extends SequentialCommandGroup {

  public TestMove(final DriveSubsystem driveSubsystem) {
    super(

      new ResetDrivetrainEncoders(driveSubsystem).withTimeout(1),

      new AutoMove(driveSubsystem, -0.35)
    );
  }
}