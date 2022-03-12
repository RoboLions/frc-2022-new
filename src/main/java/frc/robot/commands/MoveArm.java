// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArm extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final XboxController manipulatorController = RobotContainer.manipulatorController;

    public static int wrist_motion_state = 0;

    public MoveArm(ArmSubsystem arm) {
        armSubsystem = arm;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        //System.out.println(-armSubsystem.getPitch());
        double armPower = manipulatorController.getLeftY();
        boolean y = manipulatorController.getYButtonPressed();
        // y button = ground
        boolean b = manipulatorController.getBButtonPressed();
        // b button = home

        switch(wrist_motion_state) {
            case 0: // manual moving with stick
                armSubsystem.setArmPower(armPower);
                
                if(armSubsystem.armPID.deadband_active) {
                    wrist_motion_state = 0;
                }
                // if y pressed, set to ground (move down)
                if (y) {
                    wrist_motion_state = 1;
                }
                // if b pressed, set to home (move up)
                if (b) {
                    wrist_motion_state = 2;
                } 
                
                /*else if (armSubsystem.armPID.deadband_active) {
                    wrist_motion_state = 0;
                }*/
                break;
            case 1:
            // ground (y button)
                armSubsystem.setArmToGround();
                /*if(armSubsystem.armPID.deadband_active) {
                    wrist_motion_state = 0;
                }*/
                
                // if stick is full power, override Y button
                if (armPower > 0.5 || armPower < -0.5) {
                    wrist_motion_state = 0;
                }
                // if b button pressed, override Y Button
                if (b) {
                    wrist_motion_state = 2;
                }
                break;
            case 2:
            // home (b button)
                armSubsystem.setArmToHome();
                if(armSubsystem.armPID.deadband_active) {
                    wrist_motion_state = 0;
                }  
                if (y) {
                    wrist_motion_state = 1;
                }              
                break;
            default:
                wrist_motion_state = 0;
                break;
        }
        if(Math.abs(armPower) > 0.6 ) { 
            // wakes up the arm from PID control and back to joystick control
            wrist_motion_state = 0;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}