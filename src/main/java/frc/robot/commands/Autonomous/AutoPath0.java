package frc.robot.commands.Autopaths;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.AutoMove;
import frc.robot.subsystems.DriveSubsystem;

public class AutoPath0 extends CommandBase {

    public AutoPath0(final DriveSubsystem subsystem) {

        new AutoMove(subsystem, null, 3.048, 5 );

    }
    
}
