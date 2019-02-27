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

public class SetIntakePos extends Command {

  double m_desiredPos, m_desiredPosInit, m_speed, m_speedInit;

  public SetIntakePos(double desiredPos, double speed) {
    requires(Robot.m_intakeElbow);
    m_desiredPos = desiredPos;
    m_speed = Math.abs(speed);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    setTimeout(3.0);
    m_desiredPosInit = Math.min(m_desiredPos, RobotConstants.INTAKE_POT_UP);
    m_desiredPosInit = Math.max(m_desiredPos, RobotConstants.INTAKE_POT_DOWN);
    m_speedInit = Math.signum(m_desiredPosInit - Robot.m_intakeElbow.getPos())*m_speed;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_intakeElbow.setElbow(m_speedInit);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(m_speedInit > 0) {
      return Robot.m_intakeElbow.getPos() >= m_desiredPosInit;
    } else {
      return Robot.m_intakeElbow.getPos() <= m_desiredPosInit;
    }
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
