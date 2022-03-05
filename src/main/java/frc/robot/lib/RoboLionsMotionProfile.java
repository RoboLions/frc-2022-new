
package frc.robot.lib;

import frc.robot.subsystems.DriveSubsystem;

public class RoboLionsMotionProfile {
    public static double ROBO_LION_MOTION_PROFILE_START_POS = 0.0;
    public static double ROBO_LION_MOTION_PROFILE_START_VEL = 1; //1
    public static double ROBO_LION_MOTION_PROFILE_START_ACCEL = 1; //1
    public static double ROBO_LION_MOTION_PROFILE_DEFAULT_PERIOD = 0.02;
    public static double ROBO_LION_MOTION_PROFILE_DEFAULT_MAX_VELOCITY = 1; //1 // Changes: _DEFAULT_max_velocity
    public static double ROBO_LION_MOTION_PROFILE_DEFAULT_MAX_ACCELERATION = 1; //1 // Changes: _DEFAULT_max_acceleration
    public static boolean ROBO_LION_MOTION_PROFILE_STATE_ACCEL = false;
    public static boolean ROBO_LION_MOTION_PROFILE_STATE_DECEL = true;
    public double position_command;
    public double velocity_feed_forward;
    public double acceleration_feed_forward;
    public String obj_name;

    private double current_position_state; //current "state":position whether its meters or degrees
    private double current_velocity_state; //current "state":velocity whether its meters/sec or degrees/sec
    private double current_acceleration_state; //current "state":accelration whether its meters/sec*sec or degrees/sec*sec
    
    //internal limits
    private double max_acceleration; //what is our max profile allowed acceleration?
    private double max_de_celeration; //what accleration do we want to declerate at?  This uses friction as a friend
    private double max_velocity;//what is our max profile allowed velocity?
    private double time_period;//how often this function will run, used in command and state calculations

    //state switch variables
    private boolean decel_switch; //0 = not needing to decelrate, 1 = you need to decelerate
    private boolean enable_output;//0 = return with the target pos, don't update states

    //target and start positions, velocities, and accelerations
    private double target_position;
    private double target_velocity;//future use when we want to keep going at a certain speed
    private double target_acceleration;//future use when we want to be accelerating at a certain acceleration

    public RoboLionsMotionProfile() {
     init(ROBO_LION_MOTION_PROFILE_START_POS,//double start_pos
          ROBO_LION_MOTION_PROFILE_START_POS,//double target_pos
          ROBO_LION_MOTION_PROFILE_DEFAULT_MAX_VELOCITY,//double max_vel,
          ROBO_LION_MOTION_PROFILE_DEFAULT_MAX_ACCELERATION,//double max_accel
          ROBO_LION_MOTION_PROFILE_DEFAULT_PERIOD,
          ROBO_LION_MOTION_PROFILE_DEFAULT_MAX_ACCELERATION);
    }

    public RoboLionsMotionProfile(double start_pos, double target_pos, double max_vel,
                                  double max_accel, double execution_period, double deceleration) {
     init(start_pos, target_pos, max_vel, max_accel, execution_period, deceleration);
    }

    public void finalize() {
        System.out.println();
        // System.out.println("Motion Profile object deleted...");
        System.out.println();
    }

    public void name_it(String file_name) {
        obj_name = file_name;
        // System.out.println("Motion profile object named..." + obj_name);
    }

    public void command_new_target_pos(double start_pos, double target_pos) {
        //we could keep the state of our system intact and simply update our target,
        //however, for the sake of simplicity, we are going to assume the state of the system
        //is at perfect rest when this is commanded

        //pull out max dynamics and execution period
        double max_vel = max_velocity;
        double max_accel = max_acceleration;
        double period = time_period;
        double decel = max_de_celeration;

        //re-initialize our start
        init(start_pos, target_pos, max_vel, max_accel, period, decel);
    }

    public void init(double start_pos, double target_pos, double max_vel,
                     double max_accel, double execution_period, double deceleration) {
        reset();

        current_position_state = start_pos;
        target_position = target_pos;
        max_velocity = max_vel;
        max_acceleration = max_accel;
        time_period = execution_period;
        max_de_celeration = deceleration;
        enable_output = true; //important, only an initalize will enable the output
        //System.out.println("INIT HAS RUN");
    }

    public void reset() {
        position_command = 0.0; // Changes: null
        target_position = 0.0; // Changes: null
        current_position_state = ROBO_LION_MOTION_PROFILE_START_POS;
        velocity_feed_forward = 0.0; // Changes: null
        current_velocity_state = 0.0;
        acceleration_feed_forward = 0.0; // Changes: null
        current_acceleration_state = 0.0;
        max_velocity = ROBO_LION_MOTION_PROFILE_DEFAULT_MAX_VELOCITY;
        max_de_celeration = ROBO_LION_MOTION_PROFILE_DEFAULT_MAX_ACCELERATION; // Changes: null
        max_acceleration = ROBO_LION_MOTION_PROFILE_DEFAULT_MAX_ACCELERATION;
        time_period = ROBO_LION_MOTION_PROFILE_DEFAULT_PERIOD;
        decel_switch = ROBO_LION_MOTION_PROFILE_STATE_ACCEL;
        enable_output = false;
    }

    public double execute() {
        double delta_position = (target_position - current_position_state);
        double velocity_to_decel = Math.sqrt(2 * max_de_celeration*Math.abs(delta_position));
        double velocity_sign = +1.0;
        
        if (enable_output == false) {
            //position_command = 
            //current_position_state = target_position;
            velocity_feed_forward = acceleration_feed_forward = 0.0;

            // System.out.println("BEFORE RETURN OUTPUT == FALSE");
            return(position_command);
        }
        
        // FIXED
        if (velocity_to_decel < Math.abs(current_velocity_state)) { //absolute value of your current velovity state
            //our velocity to decel is less than our max, set our internal state to decel
            decel_switch = ROBO_LION_MOTION_PROFILE_STATE_DECEL;
            //once we set the decel, we continue to decel
        } 

        if (current_velocity_state == 0.0) {
            //look at target and position to compute the start
            velocity_sign = (target_position - current_position_state);
            if (velocity_sign > 0.0)
            {
                velocity_sign = +1.0;
            }
            else if (velocity_sign < 0.0)
            {
                velocity_sign = -1.0;
            }
            else
            {
                //no differnce in the command, just return without updating anything
                return(position_command);
            }
        }
        else
        {
        velocity_sign = current_velocity_state / Math.abs(current_velocity_state);
        }

    if (decel_switch == ROBO_LION_MOTION_PROFILE_STATE_ACCEL) {
        //update our velocity based on max accleration, period an integration
        //of our acceleration into the command
        current_velocity_state = current_velocity_state + velocity_sign * 
        max_acceleration * time_period;
        
        //now, if the current velocity state exceeds our acceleartion, then, we zero out our acceleration
        //and limit our velocity to the max
        if (Math.abs(current_velocity_state) >= max_velocity) {
        
            //set our velocity state to the max vel while keeping our sign intact
            current_velocity_state = velocity_sign * max_velocity;

            //zero out our acceleration state
            current_acceleration_state = 0.0;

        } else {

            //keep our acceleration state at the max
            current_acceleration_state = velocity_sign * max_acceleration;

        } 
        
    }else {
        /*
        //update our velocity based on max decleration, period an integration
        //of our acceleration into the command
        current_velocity_state = current_velocity_state -
            (velocity_sign * max_de_celeration * time_period);
        //update our state of our acceleration
        current_acceleration_state = (velocity_sign * max_de_celeration);
        */

        // keep computing the new velocity based on the decel that we 
        // already computed and the direction
        current_velocity_state = (velocity_sign * velocity_to_decel);

        //update the state of our acceleration
        current_acceleration_state = (-velocity_sign * max_de_celeration);
    }
    
    current_position_state = current_position_state + (current_velocity_state * time_period);
    /*
    System.out.println("Current Position: " + current_position_state);
    System.out.println("Current Velocity: " + current_velocity_state);
    System.out.println("Delta Position: " + delta_position);
    System.out.println("Target Decel: " + velocity_to_decel);
    System.out.println("");
    */

    //4. determine our final commands... this is where the position state may overshoot our target state,
    //when that happens, we zero our acceleration and velocity while setting our output command to our target command
    //the trick is to observe our velocity sign
    if (velocity_sign > 0.0)
    {
        //if our position overshoots the target then set our state to equal the target and zero out
        //accel and decel
        if (current_position_state >= target_position)
        {
            current_position_state = target_position;
            current_velocity_state = 0.0; // Changes: null
            current_acceleration_state = 0.0;
            enable_output = false;
        }
    }
    //look at the negative motion side
    else
    {
        //if our position overshoots the target then set our state to equal the target and zero out
        //accel and decel
        if (target_position >= current_position_state)
        {
            current_position_state = target_position;
            current_velocity_state = 0.0; // Changes: null
            current_acceleration_state = 0.0;
            enable_output = false;
        }
    }
        
        acceleration_feed_forward = current_acceleration_state;
        velocity_feed_forward = current_velocity_state;
        position_command = current_position_state;
        return(position_command); // Changes: check this
    }
}