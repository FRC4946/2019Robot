/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class SetGrabberSpeed extends Command {

  private double m_speed;

  public SetGrabberSpeed(double speed) {
    requires(Robot.m_grabber);
    m_speed = speed;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_grabber.setGrabber(m_speed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //return (Robot.m_grabber.getGrabberIn() && m_speed < 0) 
      //|| (Robot.m_grabber.getGrabberOut() && m_speed > 0);
      return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_grabber.setGrabber(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
