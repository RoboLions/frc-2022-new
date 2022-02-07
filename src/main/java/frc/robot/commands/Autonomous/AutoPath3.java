
package frc.robot.commands.Autonomous;

import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoMove;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.StopNWait;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoPath3 {
    
    public AutoPath3(final DriveSubsystem subsystem, IntakeSubsystem intakesubsystem, ShooterSubsystem shootsubsystem){

    //consider lidar aiming
      new AutoMove(subsystem, null, 2.577, 5);
      new StopNWait(subsystem, 0.2);
      new AutoIntake(intakesubsystem);
      new StopNWait(subsystem, 0.2);
      //consider aiming
      new AutoShoot(shootsubsystem);
      new StopNWait(subsystem, 0.2);
      new AutoMove(subsystem, null,6.711,5);
      new StopNWait(subsystem, 0.2);
      new AutoIntake(intakesubsystem);
      new StopNWait(subsystem, 0.2);
      //consider aiming
      new AutoShoot(shootsubsystem);
      new StopNWait(subsystem, 0.2);


    }
}
 