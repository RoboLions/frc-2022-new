/*
Link to the controller schemes: https://docs.google.com/presentation/d/1JOf1zaY09GdXTdWdXM7LIdUVHc68Tdd7p6xaXJBof7o/edit?usp=sharing

Once upon a time, an idot named Akash committed tax fraud.
Once upon a time, Ananya failed the calc test.
Once upon a time, vertically challenged Maika had to use a stool to reach the laptops.
Once upon a time, Pranav ended the world.
Once upon a time, Nate abandoned the team for half of the meetings.
Once upon a time, Katie got her ice cream from Chick-fil-a.
*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.JoystickDrive;
import frc.robot.commands.Autonomous.AutoPath1;
import frc.robot.commands.Autonomous.AutoPath2;
import frc.robot.commands.Autonomous.AutoPath3;
import frc.robot.commands.Autonomous.AutoPath4;
import frc.robot.commands.Autonomous.AutoPath5;
import frc.robot.commands.Autonomous.AutoPath6;
import frc.robot.commands.Autonomous.AutoPath7;
import frc.robot.commands.Autonomous.DefaultAutoPath;
import frc.robot.commands.Autonomous.TestPath;
import frc.robot.lib.RoboLionsPID;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.LaserSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  
  private DriveSubsystem driveSubsystem = m_robotContainer.driveSubsystem;
  private ArmSubsystem armSubsystem = m_robotContainer.armSubsystem;
  private ShooterSubsystem shooterSubsystem = m_robotContainer.shooterSubsystem;
  private LimelightSubsystem limelightSubsystem = m_robotContainer.limelightSubsystem;
  private IntakeSubsystem intakeSubsystem = m_robotContainer.intakeSubsystem;

  private final static XboxController driverController = RobotContainer.driverController;  
  
  private static final WPI_TalonFX leftBackMotor = RobotMap.leftBackDriveMotor;
  private static final WPI_TalonFX rightBackMotor = RobotMap.rightBackDriveMotor;
  private static final WPI_TalonFX leftFrontMotor = RobotMap.leftFrontDriveMotor;
  private static final WPI_TalonFX rightFrontMotor = RobotMap.rightFrontDriveMotor;

  private static final WPI_TalonFX leftShooterMotor = RobotMap.leftShooterMotor;
  private static final WPI_TalonFX rightShooterMotor = RobotMap.rightShooterMotor;

  //private static final DigitalInput elevatorSensor1 = RobotMap.elevatorSensor1;

  private RoboLionsPID drivetrainPID = m_robotContainer.driveSubsystem.positionPID;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    driveSubsystem.resetEncoders();

    leftFrontMotor.configFactoryDefault();
    rightFrontMotor.configFactoryDefault();
    leftBackMotor.configFactoryDefault();
    rightBackMotor.configFactoryDefault();
    leftShooterMotor.configFactoryDefault();
    rightShooterMotor.configFactoryDefault();

    m_robotContainer.limelightSubsystem.setDriverCamera();
    m_robotContainer.limelightSubsystem.turn_LED_OFF();

    UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setResolution(240, 180);
    camera.setFPS(12);

    m_chooser.addOption("Taxi", new DefaultAutoPath(driveSubsystem));
    m_chooser.addOption("Taxi 1", new AutoPath1(driveSubsystem, intakeSubsystem, limelightSubsystem, shooterSubsystem));
    m_chooser.setDefaultOption("Hangar 2", new AutoPath2(driveSubsystem, intakeSubsystem, limelightSubsystem, shooterSubsystem, armSubsystem));
    m_chooser.addOption("Terminal 2", new AutoPath3(driveSubsystem, intakeSubsystem, limelightSubsystem, shooterSubsystem, armSubsystem));
    //m_chooser.addOption("Guardrail 2", new AutoPath4(driveSubsystem, intakeSubsystem, limelightSubsystem, shooterSubsystem, armSubsystem));
    //m_chooser.addOption("Hangar 4", new AutoPath5(driveSubsystem, intakeSubsystem, shooterSubsystem, limelightSubsystem, armSubsystem));
    //m_chooser.addOption("Terminal 4", new AutoPath6(driveSubsystem, intakeSubsystem, shooterSubsystem, limelightSubsystem, armSubsystem));
    //m_chooser.addOption("Guardrail 4", new AutoPath7(driveSubsystem, intakeSubsystem, limelightSubsystem, shooterSubsystem, armSubsystem));
    m_chooser.addOption("Terminal 3", new TestPath(driveSubsystem, intakeSubsystem, shooterSubsystem, limelightSubsystem, armSubsystem));
    
    SmartDashboard.putData(m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    //SmartDashboard.putBoolean("Elevator Sensor 1 Value", elevatorSensor1.get());

    SmartDashboard.putNumber("Left Back F500 Temp C", leftBackMotor.getTemperature());
    SmartDashboard.putNumber("Left Front F500 Temp C", leftFrontMotor.getTemperature());
    SmartDashboard.putNumber("Right Front F500 Temp C", rightFrontMotor.getTemperature());
    SmartDashboard.putNumber("Right Back F500 Temp C", rightBackMotor.getTemperature());

    SmartDashboard.putNumber("Throttle", driverController.getLeftY());

    SmartDashboard.putNumber("Encoder MPS", shooterSubsystem.getLeftEncoderVelocityMetersPerSecond());

    SmartDashboard.putNumber("Left RPM", shooterSubsystem.getRPMOfLeftShooterWheels());
    SmartDashboard.putNumber("Right RPM", shooterSubsystem.getRPMOfRightShooterWheels());
    
    SmartDashboard.putNumber("Left Front Velocity", leftFrontMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Right Front Velocity", rightFrontMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Left Back Velocity", leftBackMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Right Back Velocity", rightBackMotor.getSelectedSensorVelocity());

    SmartDashboard.putNumber("Left Back Velocity MPS", driveSubsystem.getBackLeftEncoderVelocityMetersPerSecond());
    SmartDashboard.putNumber("Right Back Velocity MPS", driveSubsystem.getBackRightEncoderVelocityMetersPerSecond());

    SmartDashboard.putNumber("Limelight Offset", limelightSubsystem.getLimelightX());
    
    SmartDashboard.putNumber("Left Front Position", leftFrontMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Front Position", rightFrontMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Left Back Position", leftBackMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Back Position", rightBackMotor.getSelectedSensorPosition());

    SmartDashboard.putNumber("Arm Pitch", armSubsystem.getPitch());
    SmartDashboard.putNumber("Yaw", driveSubsystem.getYaw());
    //SmartDashboard.putNumber("Target Pitch", armSubsystem.target_pitch);

    SmartDashboard.putNumber("Horizontal Distance FEET From Goal", limelightSubsystem.getHorizontalDistance());
    SmartDashboard.putBoolean("WITHIN 8 TO 14 FEET?", limelightSubsystem.isWithinDistance());
    SmartDashboard.putBoolean("IS SHOOTER RAMPED UP FOR UPPER HUB W/ LL", shooterSubsystem.isShooterRampedUp());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_autonomousCommand = m_chooser.getSelected();

    drivetrainPID.reset();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    /*

    leftDrivetrainPID.initialize2(
      3.375, // Proportional Gain 4.9, 7.5, 2.205, 3.375
      17.357*0.1, // Integral Gain //12.6 42.12 ZN w FF, 17.357
      0.0, // Derivative Gain //0
      0.0, // Cage Limit 0.3 //0
      0.0, // Deadband //0
      12,// MaxOutput Volts 0.25 //100 //12
      false, //enableCage
      false //enableDeadband
    );

    // Rate Drive PID
    rightDrivetrainPID.initialize2(
      3.15, // 2.025 Proportional Gain 4.5, 7 //2.925 ZN w FF //2
      16.2*0.1, // 10.2316 Integral Gain //42.12 ZN w FF //20
      0, // Derivative Gain //0
      0.0, // Cage Limit //0.3
      0.0, // Deadband //0
      12,// MaxOutput Volts 0.25 //100 //12
      false, //enableCage
      false //enableDeadband
    );

    */
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}