/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.joystickDrive;

/**
 * Add your docs here.
 */
public class driveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private Talon m_frontRight;
  private Talon m_midRight;
  private Talon m_backRight;
  private Talon m_frontLeft;
  private Talon m_midLeft;
  private Talon m_backLeft;
  SpeedControllerGroup m_rightSide;
  SpeedControllerGroup m_leftSide;

  public driveTrain (){

    m_frontRight = new Talon (0);
    m_midRight = new Talon (1);
    m_backRight = new Talon (2);
    m_frontLeft = new Talon (3);
    m_midLeft = new Talon (4);
    m_backLeft = new Talon (5);
    //ports for the motors

    m_rightSide = new SpeedControllerGroup((SpeedController)m_frontRight, (SpeedController)m_midRight, (SpeedController)m_backRight);
    m_leftSide = new SpeedControllerGroup((SpeedController)m_frontLeft, (SpeedController)m_midLeft, (SpeedController)m_backLeft);

  }

  public void drive(double drive, double turn) {
    
    // drive method
    m_leftSide.set(drive - turn);
    m_rightSide.set(-drive - turn);
    
  }

  @Override
  public void initDefaultCommand() {

    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new joystickDrive());

  }
}
