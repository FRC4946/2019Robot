/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intakeelbow;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class SetIntakeElbow extends Command {
  
  boolean m_moveToUp;
  double m_speed;

  /**
   * Moves the elbow to the desired position
   * @param setToUp position to move the elbow to, true for up, false for down
   * @param speed speed to move to the desired position
   */
  public SetIntakeElbow(boolean setToUp, double speed) {
    requires(Robot.m_intakeElbow);
    m_moveToUp = setToUp;
    m_speed = Math.abs(speed);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(m_moveToUp) {
      Robot.m_intakeElbow.setElbow(-m_speed);
    } else {
      Robot.m_intakeElbow.setElbow(m_speed);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(m_moveToUp) {
      return Robot.m_intakeElbow.isUp();
    } else {
      return Robot.m_intakeElbow.isDown();
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_intakeElbow.setElbow(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
