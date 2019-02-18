/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.grabberarm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class SetArmToPos extends Command {

  double m_speed, m_desiredPos;
  
  public SetArmToPos(double desiredPos, double speed) {
    requires(Robot.m_grabberArm);
    m_speed = Math.abs(speed);
    m_desiredPos = desiredPos;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    m_speed = m_desiredPos < Robot.m_grabberArm.getPos() ? -m_speed : m_speed;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_grabberArm.setArm(m_speed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(m_speed < 0) {
      return Robot.m_grabberArm.getPos() <= m_desiredPos;
    } else {
      return Robot.m_grabberArm.getPos() >= m_desiredPos;
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_grabberArm.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
