/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.climber.LiftRobot;
import frc.robot.commands.drivetrain.AlignWithTarget;
import frc.robot.commands.drivetrain.JoystickDriveAbs;
import frc.robot.commands.elevator.SetElevatorJoystick;
import frc.robot.commands.grabber.SetGrabber;
import frc.robot.commands.intake.IntakeUntilBall;
import frc.robot.commands.intake.SetIntake;
import frc.robot.commands.intake.TimedIntake;
import frc.robot.commands.limelight.ToggleLimelightLED;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());

  private Joystick m_driveStick = new Joystick(RobotMap.USB_DS_DRIVESTICK);
  private Joystick m_operatorStick = new Joystick(RobotMap.USB_DS_OPERATORSTICK);
  private Button m_AButton = new JoystickButton(m_driveStick, 1);
  private Button m_BButton = new JoystickButton(m_driveStick, 2);
  private Button m_XButton = new JoystickButton(m_driveStick, 3);
  private Button m_YButton = new JoystickButton(m_driveStick, 4);
  private Button m_LBButton = new JoystickButton(m_driveStick, 5);
  private Button m_RBButton = new JoystickButton(m_driveStick, 6);
  private Button m_ViewButton = new JoystickButton(m_driveStick, 7);
  private Button m_StartButton = new JoystickButton(m_driveStick, 8);
  private Button m_LeftStickButton = new JoystickButton(m_driveStick, 9);
  private Button m_RightStickButton = new JoystickButton(m_driveStick, 10);


  public Joystick getDriveStick() {
    return m_driveStick;
  }

  public OI() {

    // TODO: Bind buttons to commands
    m_BButton.whileHeld(new JoystickDriveAbs());
    m_StartButton.whenPressed(new ToggleLimelightLED());
    m_XButton.whenPressed(new AlignWithTarget());
    m_RBButton.whenPressed(new IntakeUntilBall());
    m_LBButton.whenReleased(new SetIntake(0.1));
    m_AButton.whenPressed(new SetGrabber(true, 0.1));
    m_YButton.whenPressed(new SetGrabber(false, 0.1));
    m_LeftStickButton.whileHeld(new LiftRobot(0.2));
    m_RightStickButton.whileHeld(new LiftRobot(-0.2));

    //m_XButton.whenPressed(new SetGrabber(true, 0.1));
    //m_YButton.whenPressed(new SetGrabber(false, 0.1));

  }
}
