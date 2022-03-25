// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootShooter extends CommandBase {

  private final static XboxController manipulatorController = RobotContainer.manipulatorController;
  private final static XboxController driverController = RobotContainer.driverController;
  private final ShooterSubsystem shooterSubsystem;

  //private static final DigitalInput elevatorSensor1 = RobotMap.elevatorSensor1;
  
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
    
    // Distances based on reading on LL, shooter speed, hood speed
    // 13, 1.06, 0.85
    // 12, 1, 0.8
    // 11, 1, 0.75
    // 10, 1, 0.7
    // 9, 1, 0.62
    // 8, 

    // minus 1.54 feet to account for bumper and distance from front of robot to limelight
    //double speed = 1.74 + 0.0116 * (LimelightSubsystem.getHorizontalDistance() - 1.54) + 0.00684 * (LimelightSubsystem.getHorizontalDistance() - 1.54) * (LimelightSubsystem.getHorizontalDistance() - 1.54);
    double speed = 1.07;
    // 1.4 = 0.38 percent command
    // 0.28 / 0.9
    double hoodSpeed = 0.55;

    if (driverController.getAButtonPressed()) {
      LimelightSubsystem.setVisionProcessor();
    } 
    if (driverController.getYButtonPressed()) {
      LimelightSubsystem.setDriverCamera();
    }

    // reverse
    if (manipulatorController.getLeftTriggerAxis() > 0.25) {
      shooterSubsystem.setSpeed(-0.35);
    } 
    // shoot upper hub
    else if (manipulatorController.getRightTriggerAxis() > 0.25) {
      shooterSubsystem.steadyShoot(speed); //0.94
      shooterSubsystem.setHoodSpeed(hoodSpeed);
    } 
    // shoot low goal
    else if (manipulatorController.getRightBumper()) {
      shooterSubsystem.steadyShoot(0.88);
    }
    // shoot without limelight to upper hub from launchpad
    else if (manipulatorController.getLeftBumper()) {
      shooterSubsystem.steadyShoot(2.2); //2.2
      shooterSubsystem.setHoodSpeed(hoodSpeed);
    }
    else {
      shooterSubsystem.stopShooter();
      shooterSubsystem.lastShootVelocity = 0;
    }

    if (manipulatorController.getXButton()) {
      shooterSubsystem.moveBeltUp();
    } else if (manipulatorController.getAButton()) {
      shooterSubsystem.moveBeltDown();
    } else {
      shooterSubsystem.stopBelt();
    }

    /*if (manipulatorController.getXButton()) {
      if (Math.abs(shooterSubsystem.getLeftEncoderVelocityMetersPerSecond()) < 0.25) {
        if (elevatorSensor1.get() == false) {
          shooterSubsystem.stopBelt();
        } else {
          shooterSubsystem.moveBeltUp();
        }
      } else {
        shooterSubsystem.moveBeltUp();
      }
    } else if (manipulatorController.getAButton()) {
      shooterSubsystem.moveBeltDown();
    } else {
      shooterSubsystem.stopBelt();
    }*/
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