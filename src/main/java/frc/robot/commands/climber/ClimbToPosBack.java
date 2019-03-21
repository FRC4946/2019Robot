/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ClimbToPosBack extends Command {

  double m_height, m_heightInit;
  boolean m_isMovingUp;

  public ClimbToPosBack(double height) { 
    requires(Robot.m_climber);
    m_height = height;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() { 
    m_heightInit = m_height;
    m_isMovingUp = Robot.m_climber.getBackClimberHeight() < m_heightInit;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(m_isMovingUp) {
      Robot.m_climber.setBack(0.5);
    } else {
      Robot.m_climber.setBack(-0.5);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(m_isMovingUp) {
      return Robot.m_climber.getBackClimberHeight() >= m_heightInit;
    } else {
      return Robot.m_climber.getBackClimberHeight() <= m_heightInit;
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_climber.stopClimber();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.m_climber.stopClimber();
  }
}
