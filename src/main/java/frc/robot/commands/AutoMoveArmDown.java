// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class AutoMoveArmDown extends CommandBase {
    private final ArmSubsystem armSubsystem;

    public static boolean end_me;

    public AutoMoveArmDown(ArmSubsystem arm) {
        armSubsystem = arm;
        end_me = false;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        
      armSubsystem.setArmToGround();
      /*if(armSubsystem.armPID.deadband_active) {
          end_me = true;
      }*/
    }

    @Override
    public boolean isFinished() {
        return end_me;
    }
}