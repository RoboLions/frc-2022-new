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

    // AT 6 FEET (AUTO PATH 2 1ST SHOT) - NO HOOD, 1.85 SHOOTER
    // AT 6.9 FEET (AUTO PATH 2ND SHOT) - 0.39, 1.5 SHOOTER
    
    // Distances based on reading on LL, shooter speed, hood speed 
    // 16, 1.06, 0.93
    // 15, 1, 0.88
    // 14, 1, 1.05, 0.83
    // 13, 1.06, 0.85
    // 12, 1, 0.8
    // 11, 1, 0.75
    // 10, 1, 0.7
    // 9, 1, 0.62
    // 8, 1.07, 0.55
    // 7, 1.15, 0.38 // not consistent
    // 6, 1.85, 0

    // 6, 1.85, 0
    // 7, 1.5, 0.4
    // 7.5, 1.55, 0.4
    // 8, 1.6, 0.4
    // 8.5 1.63, 0.4
    // 9, 1.65, 0.4
    // 9.5, 1.67, 0.4
    // 10, 1.69, 0.4
    // 10.5, 1.71, 0.4
    // 11, 1.73, 0.4
    // 11.5, 1.75, 0.5
    // 12, 1.8, 0.5
    // 12.5, 1.85, 0.58
    // 13, 1.87, 0.7
    // 13.5, 1.9, 0.7
    // 14, 1.95, 0.8


    // minus 1.54 feet to account for bumper and distance from front of robot to limelight
    //double speed = 1.74 + 0.0116 * (LimelightSubsystem.getHorizontalDistance() - 1.54) + 0.00684 * (LimelightSubsystem.getHorizontalDistance() - 1.54) * (LimelightSubsystem.getHorizontalDistance() - 1.54);
    //double speed = 1.59; // 1.6
    //10.4 + -2.6x + 0.235x^2 + -6.77E-03x^3
    double x = LimelightSubsystem.getHorizontalDistance();
    //double speed = (10.4) + (-2.6 * x) + (0.235 * Math.pow(x, 2)) + (-0.00677 * Math.pow(x, 3));
    //double hoodSpeed = 1; //0
    // -6.3 + 1.79x + -0.148x^2 + 4.07E-03x^3
    //double hoodSpeed = (-6.3) + (1.79 * x) + (-0.148 * Math.pow(x, 2)) + (0.00407 * Math.pow(x, 3));
    double hoodSpeed = 0;
    double speed = 0.0562*x + 1.13; //0.0581*x + 1.11;
    if (x >= 7 && x <= 11) {
      hoodSpeed = 0.4;
    } else if (x > 11 && x < 11.5) {
      hoodSpeed = 0.2*x - 1.8;
    } else if (x >= 11.5 && x <= 12) {
      hoodSpeed = 0.5;
    } else if (x > 12 && x < 13) {
      hoodSpeed = 0.2*x - 1.91;
    } else if (x >= 13 && x <= 13.5) {
      hoodSpeed = 0.7;
    } else if (x > 13.5 && x <= 14.5) { //14
      hoodSpeed = 0.07*x - 0.243; //0.2*x - 2;
    } else {
      hoodSpeed = 0;
    }
    //double speed = 1.96;
    //hoodSpeed = 0.85;

    //System.out.println(speed + ", " + hoodSpeed);

    if (hoodSpeed > 1) {
      hoodSpeed = 1;
    } else if (hoodSpeed < -1) {
      hoodSpeed = -1;
    }

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
      shooterSubsystem.steadyShoot(0.83);
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