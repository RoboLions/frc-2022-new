
package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignShooter;
//import frc.robot.commands.AlignWithLIDAR;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoMove;
import frc.robot.commands.AutoMoveAndIntake;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.StopNWait;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
//import frc.robot.subsystems.LIDARLiteSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SimpleShooterSubsystem;

public class AutoPath3 extends SequentialCommandGroup {
    
  /** 
   * pre-loaded with 1 ball, move forward to intake ball, shoot both balls,
   * turn to terminal, move forward to intake ball at terminal, 
   * move within range of hub for LL, shoot ball
  */

  public AutoPath3(final DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, 
  ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem){
    super(

      // intake facing away from hub, move forwards and intake ball
      new AutoMoveAndIntake(driveSubsystem, intakeSubsystem, 0.5),

      new StopNWait(driveSubsystem, 0.5),

      // Target hub
      new AlignShooter(limelightSubsystem, driveSubsystem),

      new StopNWait(driveSubsystem, 0.5),

      // Shoot balls
      new AutoShoot(shooterSubsystem).withTimeout(6),

      // Move forward to terminal with intake running
      new AutoMoveAndIntake(driveSubsystem, intakeSubsystem, 2.7),

      new StopNWait(driveSubsystem, 0.5),

      // Align to hub
      new AlignShooter(limelightSubsystem, driveSubsystem),

      // Shoot
      new AutoShoot(shooterSubsystem).withTimeout(6)
    );
  }
}
 