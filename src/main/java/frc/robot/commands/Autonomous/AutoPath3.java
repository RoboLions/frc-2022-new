
package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.AlignShooter;
//import frc.robot.commands.AlignWithLIDAR;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoMove;
import frc.robot.commands.AutoMoveElevator;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.AutoTurnLLOn;
import frc.robot.commands.StopNWait;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
//import frc.robot.subsystems.LIDARLiteSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SimpleShooterSubsystem;

public class AutoPath3 extends SequentialCommandGroup {
    
  /** 
   * start from position farthest from the hanger zone (long linear line to terminal)
   * pre-loaded with 1 ball, move forward to intake ball, shoot both balls,
   * turn to terminal, move forward to intake ball at terminal, 
   * move within range of hub for LL, shoot ball
  */

  public AutoPath3(final DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, 
  ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem){
    super(

      // intake facing away from hub, move forwards part of the way
      new AutoMove(driveSubsystem, 0.9),

      // intake ball with elevator going
      new ParallelRaceGroup(
        new AutoMove(driveSubsystem, 0.1),
        new AutoIntake(intakeSubsystem),
        new AutoMoveElevator(shooterSubsystem)
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
      new AutoTurn(driveSubsystem, 5),

      new StopNWait(driveSubsystem, 0.5),

      // Move forward to terminal but not all the way
      new AutoMove(driveSubsystem, 2.1),

      new StopNWait(driveSubsystem, 0.2),

      // Finish move forward to terminal with intake running
      new ParallelRaceGroup(
        new AutoMove(driveSubsystem, 0.1),
        new AutoIntake(intakeSubsystem),
        new AutoMoveElevator(shooterSubsystem)
      ),

      new StopNWait(driveSubsystem, 0.5),

      // Move back to get in range of LL
      new AutoMove(driveSubsystem, -1),

      // Align to hub
      //new AlignShooter(limelightSubsystem, driveSubsystem),

      // Shoot
      new AutoShoot(shooterSubsystem).withTimeout(4)
    );
  }
}