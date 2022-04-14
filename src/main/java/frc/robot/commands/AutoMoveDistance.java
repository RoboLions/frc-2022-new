// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.lib.RoboLionsPID;
import frc.robot.subsystems.DriveSubsystem;

// this function is designed to move a specified distance forward/backward
public class AutoMoveDistance extends CommandBase {
        private final DriveSubsystem drivesubsystem;
        private double start_absolute_position_meters; // this is our starting position on a move (the encoder reading)
        private double target_absolute_position_meters; // this is our target position
        private double distance_to_move; // because Dustin made us

        public AutoMoveDistance(final DriveSubsystem subsystem, Mode mode, double distance_in_meters, double speed) {
                drivesubsystem = subsystem;
                addRequirements(drivesubsystem);
                start_absolute_position_meters = drivesubsystem.distanceTravelledinMeters();
                //target_absolute_position_meters = distance_in_meters; DJK says to do this 4/13/22
                target_absolute_position_meters = distance_in_meters + start_absolute_position_meters;
                distance_to_move = distance_in_meters;
        }

        // this is the function we mainly use
        public AutoMoveDistance(final DriveSubsystem subsystem, double distance) {
                drivesubsystem = subsystem;
                addRequirements(drivesubsystem);
                // (start position in meters) this is absolute
                start_absolute_position_meters = drivesubsystem.distanceTravelledinMeters(); 
                //target_absolute_position_meters = distance; DJK says to do this 4/13/22
                target_absolute_position_meters = distance + start_absolute_position_meters;
                distance_to_move = distance;
        }

        @Override
        public void initialize() {
                //drivesubsystem.resetEncoders();
                //drivesubsystem.ZeroYaw();
                drivesubsystem.positionPID.reset();
                drivesubsystem.positionMotionProfile.reset();
                start_absolute_position_meters = drivesubsystem.distanceTravelledinMeters();
                drivesubsystem.state_flag_motion_profile = true;
        }

        @Override
        public void execute() { 
                // This function is constantly being called in the class at 50 Hz
                // This implements a straight move without any heading control 
                //double position_profile_command = drivesubsystem.positionMotionProfile.execute();
                //double feed_forward_rate = drivesubsystem.positionMotionProfile.velocity_feed_forward;                
                //System.out.println("Command: " + position_profile_command + " , FF: " + feed_forward_rate);
                drivesubsystem.autoDrive(distance_to_move, 0.0); //TODO pls check parameter
                // System.out.println("AUTO WORKS");
        }

        @Override
        public boolean isFinished() {
                // This function is constantly being called in the class at 50 Hz
                // This helps to determine when you are done with the command
                //boolean tempReturn = false;
                double distance_driven = drivesubsystem.distanceTravelledinMeters() - start_absolute_position_meters;
                double positionError = Math.abs(target_absolute_position_meters - distance_driven);
                return(positionError < 0.01); // stop whenever we go the commanded distance within 1 cm
                //return(tempReturn);
        } 

        public enum Mode {
                DISTANCE, TIME
        }
}	