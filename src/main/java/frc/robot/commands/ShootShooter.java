// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootShooter extends CommandBase {

  private final static XboxController manipulatorController = RobotContainer.manipulatorController;
  private final ShooterSubsystem shooterSubsystem;
  
  public ShootShooter(ShooterSubsystem shooter) {
    shooterSubsystem = shooter;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.stopShooter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // function: 0.0866x + 1.6
    // after adding in P value:
    // 0.82 good for low goal
    // 1.76 good for 1 feet
    // 1.82 good for 2 feet
    // 1.84 good for 3 feet
    // 1.87 good for 4 feet
    // 2 good for 6 feet
    // 2.38 good for 8 feet
    // 2.5 good for 10 feet

    // minus 1.54 feet to account for bumper and distance from front of robot to limelight
    double speed = 1.74 + 0.0116 * (LimelightSubsystem.getHorizontalDistance() - 1.54) + 0.00684 * (LimelightSubsystem.getHorizontalDistance() - 1.54) * (LimelightSubsystem.getHorizontalDistance() - 1.54);

    if (manipulatorController.getAButton()) {
      LimelightSubsystem.setVisionProcessor();
    } else {
      LimelightSubsystem.setDriverCamera();
    }

    if (manipulatorController.getXButton()) {
      shooterSubsystem.moveBeltUp();
      shooterSubsystem.steadyShoot(speed);
    } else if (manipulatorController.getBButton()) {
      shooterSubsystem.moveBeltDown();
    } else {
      shooterSubsystem.stopBelt();
      shooterSubsystem.stopShooter();
    }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}