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
import frc.robot.commands.climber.DropClimber;
import frc.robot.commands.climber.MidStageClimb;
import frc.robot.commands.climber.SetBackClimber;
import frc.robot.commands.climber.SetFrontClimber;
import frc.robot.commands.drivetrain.StrafeToTarget;
import frc.robot.commands.elevator.MoveToLevel2Rocket;
import frc.robot.commands.elevator.MoveToLevel3Rocket;
import frc.robot.commands.elevator.MoveToLowHeight;
import frc.robot.commands.elevator.SetElevatorToAboveElbow;
import frc.robot.commands.grabber.ToggleGrabber;
import frc.robot.commands.grabberarm.SetArmToPos;
import frc.robot.commands.intakeelbow.SetElbowSpeed;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  private Joystick m_driveStick = new Joystick(RobotMap.USB_DS_DRIVESTICK);
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

  private Joystick m_operatorStick = new Joystick(RobotMap.USB_DS_OPERATORSTICK);
  private Button m_AButtonOperator = new JoystickButton(m_operatorStick, 1);
  private Button m_BButtonOperator = new JoystickButton(m_operatorStick, 2);
  private Button m_XButtonOperator = new JoystickButton(m_operatorStick, 3);
  private Button m_YButtonOperator = new JoystickButton(m_operatorStick, 4);
  private Button m_LBButtonOperator = new JoystickButton(m_operatorStick, 5);
  private Button m_RBButtonOperator = new JoystickButton(m_operatorStick, 6);
  private Button m_ViewButtonOperator = new JoystickButton(m_operatorStick, 7);
  private Button m_StartButtonOperator = new JoystickButton(m_operatorStick, 8);
  private Button m_LeftStickButtonOperator = new JoystickButton(m_operatorStick, 9);
  private Button m_RightStickButtonOperator = new JoystickButton(m_operatorStick, 10);

  public Joystick getDriveStick() {
    return m_driveStick;
  }

  public Joystick getOperatorStick() {
    return m_operatorStick;
  }

  //NEGATIVE IS DOWN ON THE ELEVATOR
  //NEGATIVE IS OUT ON THE GRABBER ARM
  //NEGATIVE IS TOWARDS THE GROUND ON THE CLIMBER
  //NEGATIVE IS DOWN ON THE ELBOW
  //NEGATIVE IS IN ON THE GRABBER (POSITIVE WILL MAKE IT GRAB THE HATCH)
  //NEGATIVE IS IN ON THE INTAKE
  //WITHOUT NEGATION, DOWN ON THE LEFT JOYSTICK ANALOG IS UP ON THE ELEVATOR

  //TODO: CLIMBER LIM SWITCHES NORMALLY CLOSED

  public OI() { 
    //----- operator stick
    m_YButtonOperator.whenPressed(new MoveToLevel3Rocket());
    m_BButtonOperator.whenPressed(new MoveToLevel2Rocket());
    m_XButtonOperator.whenPressed(new SetElevatorToAboveElbow());
    m_AButtonOperator.whenPressed(new MoveToLowHeight());
    //m_LeftStickButtonOperator.whenPressed(new OverrideElevator());

    //m_LBButtonOperator.whileHeld(new SetDriveTrain(0.2));
    //m_LBButtonOperator.whileHeld(new SetIntakeSpeed(-0.99));
    //m_RBButtonOperator.whileHeld(new SetDriveTrain(-0.2));
    //m_RBButtonOperator.whileHeld(new SetDriveTrain(0.99));

    
    m_LBButtonOperator.whileHeld(new SetBackClimber(0.4));
    m_RBButtonOperator.whileHeld(new SetFrontClimber(0.3));

    //----- driver stick
    m_AButton.whileHeld(new SetElbowSpeed(0.2));
    m_YButton.whileHeld(new SetElbowSpeed(-0.2));

    m_XButton.whenPressed(new SetArmToPos(RobotConstants.GRABBER_ARM_OUT, 0.6));
    m_BButton.whenPressed(new SetArmToPos(RobotConstants.GRABBER_ARM_HOLD_HATCH, 0.6));

    m_LBButton.whileHeld(new StrafeToTarget());
    m_RBButton.whenPressed(new ToggleGrabber(0.9));

    m_StartButton.whileHeld(new DropClimber());
    m_ViewButton.whileHeld(new MidStageClimb());
  }
}
