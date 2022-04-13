package frc.robot.commands.Autonomous;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class FiveBallAuto extends SequentialCommandGroup{

    private static Trajectory[] trajectories = {
        PathPlanner.loadPath("First 3", 8, 5),
        PathPlanner.loadPath("Human Player 1", 8, 5),
        PathPlanner.loadPath("Shooting Location", 8, 5)
    };

    public FiveBallAuto(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, 
    LimelightSubsystem limelightSubsystem, ShooterSubsystem shooterSubsystem, ArmSubsystem armSubsystem) {
        super(
            new ParallelCommandGroup(
                new AutoMoveArmDown(armSubsystem).withTimeout(1), 
                new ResetDrivetrainEncoders(driveSubsystem).withTimeout(1),
                new AutoTurnLLOn(limelightSubsystem).withTimeout(1)
            ),
            new FollowTrajectory(driveSubsystem, trajectories[0]).deadlineWith(new AutoIntake(intakeSubsystem)),
            new ParallelCommandGroup(
                new AutoShoot(shooterSubsystem),
                new AutoMoveElevatorUp(shooterSubsystem)
            ).withTimeout(4),
            new ParallelCommandGroup(
                new FollowTrajectory(driveSubsystem, trajectories[1]),
                new WaitCommand(2)
            ).deadlineWith(new AutoIntake(intakeSubsystem)),
            new FollowTrajectory(driveSubsystem, trajectories[2]),
            new ParallelCommandGroup(
                new AutoShoot(shooterSubsystem),
                new AutoMoveElevatorUp(shooterSubsystem)
            ).withTimeout(4)
        );
    }
}
