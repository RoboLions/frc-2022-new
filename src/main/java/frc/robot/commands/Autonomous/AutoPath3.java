
package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignShooter;
import frc.robot.commands.AlignWithLIDAR;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoMove;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.StopNWait;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LIDARLiteSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SimpleShooterSubsystem;

public class AutoPath3 extends SequentialCommandGroup {
    
  /** 
   * TODO: path description here
  */

  public AutoPath3(final DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, 
  ShooterSubsystem shootSubsystem, LIDARLiteSubsystem lidar, LimelightSubsystem limelightSubsystem){
    super(
      // TODO: consider lidar aiming
      // Move forward
      new AutoMove(driveSubsystem, null, 2.577, 5),
      new StopNWait(driveSubsystem, 0.2),

      // Intake a ball
      new AutoIntake(intakeSubsystem),
      new StopNWait(driveSubsystem, 0.2),

      // Align to hub
      new AlignShooter(limelightSubsystem, driveSubsystem),

      // Shoot balls 
      new AutoShoot(shootSubsystem),
      new StopNWait(driveSubsystem, 0.2),

      // Move forward
      new AutoMove(driveSubsystem, null,6.711,5),
      new StopNWait(driveSubsystem, 0.2),

      // Intake ball
      new AutoIntake(intakeSubsystem),
      new StopNWait(driveSubsystem, 0.2),

      // ALign to hub
      new AlignShooter(limelightSubsystem, driveSubsystem),

      // Shoot
      new AutoShoot(shootSubsystem),
      new StopNWait(driveSubsystem, 0.2)
    );
  }
}
 