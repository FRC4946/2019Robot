/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.autonomous.TestAuto;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.GrabberArm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeElbow;
import frc.robot.subsystems.Limelight;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static DriveTrain m_driveTrain = new DriveTrain();
  public static Limelight m_limelight = new Limelight();
  public static Climber m_climber = new Climber();
  public static Grabber m_grabber = new Grabber();
  public static Intake m_intake = new Intake();
  public static Elevator m_elevator = new Elevator();
  public static IntakeElbow m_intakeElbow = new IntakeElbow();
  public static GrabberArm m_grabberArm = new GrabberArm();
  public static OI m_oi = new OI();

  Command m_autonomousCommand;
  private Preferences m_robotPrefs;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    CameraServer.getInstance().startAutomaticCapture();

    m_robotPrefs = Preferences.getInstance();

    initializeSmartDashboard();
    RobotConstants.repopulatePrefs(m_robotPrefs);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    updateSmartDashboard();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    RobotConstants.updatePrefs(m_robotPrefs);
    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    System.out.println(m_chooser.getSelected().getName());
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    RobotConstants.updatePrefs(m_robotPrefs);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  /**
   * Sets up smart dashboard
   */
  public void initializeSmartDashboard() {

    //auto stuff
    m_chooser.setDefaultOption("Test Auto", new TestAuto()); // TODO: Implement autonomous command
    SmartDashboard.putData("Autonomous/Auto Mode", m_chooser);

    //Drive initialization
    SmartDashboard.putNumber("Drive/Gyro", m_driveTrain.getGyroAngle());
    SmartDashboard.putNumber("Drive/Right Front Encoder", m_driveTrain.getRightFrontEncDistance());
    SmartDashboard.putNumber("Drive/Left Front Encoder", m_driveTrain.getLeftFrontEncDistance());
    SmartDashboard.putNumber("Drive/Right Back Encoder", m_driveTrain.getRightBackEncDistance());
    SmartDashboard.putNumber("Drive/Left Back Encoder", m_driveTrain.getLeftBackEncDistance());

    //elevator initialization
    SmartDashboard.putNumber("Elevator/Height", m_elevator.getHeight());

    //outer intake
    SmartDashboard.putNumber("Intake/Outer/Elbow Angle", m_intakeElbow.getPos());

    //grabber
    SmartDashboard.putNumber("Grabber/Grabber Arm/Position", m_grabberArm.getPos());
    SmartDashboard.putBoolean("Grabber/Grabber In", m_grabber.getGrabberIn());
    SmartDashboard.putBoolean("Grabber/Has Hatch", m_grabber.getGrabberOut());

    //climber
    SmartDashboard.putNumber("Climber/Front Climber Position", m_climber.getFrontClimberHeight());
    SmartDashboard.putNumber("Climber/Back Climber Position", m_climber.getBackClimberHeight());
   }

  /**
   * Send info to smart dashboard
   */
  public void updateSmartDashboard() {
    //Drive
    SmartDashboard.getEntry("Drive/Gyro").setDouble(m_driveTrain.getGyroAngle());
    SmartDashboard.getEntry("Drive/Right Front Encoder").setDouble(m_driveTrain.getRightFrontEncDistance());
    SmartDashboard.getEntry("Drive/Left Front Encoder").setDouble(m_driveTrain.getLeftFrontEncDistance());
    SmartDashboard.getEntry("Drive/Right Back Encoder").setDouble(m_driveTrain.getRightBackEncDistance());
    SmartDashboard.getEntry("Drive/Left Back Encoder").setDouble(m_driveTrain.getLeftBackEncDistance());;

    //Elevator
    SmartDashboard.getEntry("Elevator/Height").setDouble(m_elevator.getHeight());

    //Outer intake
    SmartDashboard.getEntry("Intake/Outer/Elbow Angle").setDouble(m_intakeElbow.getPos());

    //grabber
    SmartDashboard.getEntry("Grabber/Grabber Arm/Position").setDouble(m_grabberArm.getPos());
    SmartDashboard.getEntry("Grabber/Grabber In").setBoolean(m_grabber.getGrabberIn());
    SmartDashboard.getEntry("Grabber/Has Hatch").setBoolean(m_grabber.getGrabberOut());

    //climber
    SmartDashboard.getEntry("Climber/Front Climber Position").setDouble(m_climber.getFrontClimberHeight());
    SmartDashboard.getEntry("Climber/Back Climber Position").setDouble(m_climber.getBackClimberHeight());
  }

}
