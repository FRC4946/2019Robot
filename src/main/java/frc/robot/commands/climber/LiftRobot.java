/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class LiftRobot extends Command {

  private double m_frontSpeed, m_backSpeed;

  public LiftRobot() {
    requires(Robot.m_climber);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if(Robot.m_oi.getOperatorStick().getPOV() == 0) {

      Robot.m_climber.setFront(0.4);
      Robot.m_climber.setBack(0.3);

    } else if (Robot.m_oi.getOperatorStick().getPOV() == 90) {

      Robot.m_climber.setFront(-0.2);

    } else if (Robot.m_oi.getOperatorStick().getPOV() == 180) {

      Robot.m_climber.setFront(-0.2);
      Robot.m_climber.setBack(-0.45);

    } else if (Robot.m_oi.getOperatorStick().getPOV() == 270) {

      Robot.m_climber.setBack(-0.45);

    } else {
      Robot.m_climber.setClimber(0);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.m_climber.isClimberTopped();
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
    end();
  }
}
