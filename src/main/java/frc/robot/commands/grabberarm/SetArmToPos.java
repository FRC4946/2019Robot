/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.grabberarm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConstants;

/**
 * Sets the arm to the desired position
 */
public class SetArmToPos extends Command {

  double m_speed, m_desiredPos, m_desiredPosInit, m_speedInit;
  
  public SetArmToPos(double desiredPos, double speed) {
    requires(Robot.m_grabberArm);
    m_speed = speed; 
    m_desiredPos = desiredPos;
    this.setTimeout(0.6);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() { //pressing a button calls this everytime not constructor - mao
    if(Robot.m_grabber.getGrabberOut() && m_desiredPos < RobotConstants.GRABBER_ARM_HOLD_HATCH) {
      m_desiredPosInit = RobotConstants.GRABBER_ARM_HOLD_HATCH;
    } else {
      m_desiredPosInit = m_desiredPos;
    }
    m_speedInit = Math.signum(Robot.m_grabberArm.getPos() - m_desiredPos)*Math.abs(m_speed); 
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_grabberArm.setArm(m_speedInit);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(m_speedInit < 0) { //moving out
      return Robot.m_grabberArm.getPos() >= m_desiredPosInit;
    } else {
      //return Robot.m_grabber.getGrabberOut() && Robot.m_grabberArm.getPos() <= RobotConstants.GRABBER_ARM_HOLD_HATCH || Robot.m_grabberArm.getPos() <= m_desiredPos;
      return Robot.m_grabberArm.getPos() <= m_desiredPosInit;
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
