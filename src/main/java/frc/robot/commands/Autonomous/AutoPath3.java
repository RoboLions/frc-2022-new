package frc.robot.commands.Autopaths;

import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoMove;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.StopNWait;
import frc.robot.subsystems.DriveSubsystem;

public class AutoPath3 {
    
    public AutoPath3(final DriveSubsystem subsystem){

    //consider lidar aiming
      new AutoMove(subsystem, null, 2.577, 5);
      new StopNWait(subsystem, 0.2);
      new AutoIntake(true);
      new StopNWait(subsystem, 0.2);
      //consider aiming
      new AutoShoot(true);
      new StopNWait(subsystem, 0.2);
      new AutoMove(subsystem, null,6.711,5);
      new StopNWait(subsystem, 0.2);
      new AutoIntake(true);
      new StopNWait(subsystem, 0.2);
      //consider aiming
      new AutoShoot(true);
      new StopNWait(subsystem, 0.2);


    }
}
 