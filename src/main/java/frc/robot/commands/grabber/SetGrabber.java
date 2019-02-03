/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class SetGrabber extends Command {

  private boolean m_setIn;
  private double m_speed;

  public SetGrabber(boolean setIn, double speed) {
    requires(Robot.m_grabber);
    m_setIn = setIn;
    m_speed = Math.abs(speed);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(m_setIn) {
      Robot.m_grabber.setGrabber(m_speed);
    } else {
      Robot.m_grabber.setGrabber(-m_speed);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(m_setIn) {
      return Robot.m_grabber.getGrabberIn();
    } else {
      return Robot.m_grabber.getGrabberOut();
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_grabber.setGrabber(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
