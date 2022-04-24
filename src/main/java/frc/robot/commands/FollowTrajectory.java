// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class FollowTrajectory extends CommandBase {

  private Trajectory trajectory;
  private DriveSubsystem drivetrain;

  private static RamseteController controller;
  private static Timer timer = new Timer();




  /** Creates a new FollowTrajectory. */
  public FollowTrajectory(DriveSubsystem drivetrain, Trajectory trajectory) {
    addRequirements(drivetrain);

    this.trajectory = trajectory;
    this.drivetrain = drivetrain;


    controller = new RamseteController();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds speeds = controller.calculate(drivetrain.getPose(), trajectory.sample(timer.get()));
    DifferentialDriveWheelSpeeds wheelSpeeds = DriveConstants.KINEMATICS.toWheelSpeeds(speeds);
    double left = wheelSpeeds.leftMetersPerSecond;
    double right = wheelSpeeds.rightMetersPerSecond;    

    drivetrain.straightDrive(left, right);
    SmartDashboard.putBoolean("Auto?", true);
    SmartDashboard.putNumber("timer", timer.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    SmartDashboard.putBoolean("Auto?", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > trajectory.getTotalTimeSeconds();
  }
}