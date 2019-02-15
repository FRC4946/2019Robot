/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intakeelbow;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConstants;

/**
 * Moves the elbow to the desired position
 * @param setToUp position to move the elbow to, true for up, false for down
 * @param speed speed to move to the desired position
 */
public class SetIntakeElbow extends Command {

  boolean m_setUp;
  double m_speed, m_desiredValue;

  public SetIntakeElbow(boolean setUp, double speed) {
    requires(Robot.m_intakeElbow);
    m_setUp = setUp;
    m_speed = Math.abs(speed);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    if(m_setUp) {
      m_desiredValue = Robot.m_intakeElbow.getPos() < RobotConstants.INTAKE_POT_BALL_HEIGHT 
        ? RobotConstants.INTAKE_POT_BALL_HEIGHT : RobotConstants.INTAKE_POT_UP;
    } else {
      m_desiredValue = Robot.m_intakeElbow.getPos() > RobotConstants.INTAKE_POT_BALL_HEIGHT 
        ? RobotConstants.INTAKE_POT_BALL_HEIGHT : RobotConstants.INTAKE_POT_DOWN;
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_intakeElbow.setElbow(Math.signum(m_desiredValue - Robot.m_intakeElbow.getPos())*m_speed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Math.abs(Robot.m_intakeElbow.getPos() - m_desiredValue) <= 2;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_intakeElbow.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
