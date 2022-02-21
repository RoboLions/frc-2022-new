
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
   * pre-loaded with 1 ball, move forward to intake ball, 
  */

  public AutoPath3(final DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, 
  ShooterSubsystem shootSubsystem, LIDARLiteSubsystem lidar, LimelightSubsystem limelightSubsystem){
    super(

      // Target ball on field
      new AlignWithLIDAR(lidar, driveSubsystem), 

      // Move forward
      new AutoMove(driveSubsystem, 2.7),
      new StopNWait(driveSubsystem, 0.5),

      // Intake a ball
      new AutoIntake(intakeSubsystem).withTimeout(2),
      new StopNWait(driveSubsystem, 0.5),

      // Align to hub
      new AlignShooter(limelightSubsystem, driveSubsystem),

      // Shoot balls 
      new AutoShoot(shootSubsystem).withTimeout(6),
      new StopNWait(driveSubsystem, 0.5),

      // Move forward
      new AutoMove(driveSubsystem, 2.7),
      new StopNWait(driveSubsystem, 0.5),

      // Intake ball
      new AutoIntake(intakeSubsystem).withTimeout(2),
      new StopNWait(driveSubsystem, 0.5),

      // ALign to hub
      new AlignShooter(limelightSubsystem, driveSubsystem),

      // Shoot
      new AutoShoot(shootSubsystem).withTimeout(6),
      new StopNWait(driveSubsystem, 0.5)
    );
  }
}
 