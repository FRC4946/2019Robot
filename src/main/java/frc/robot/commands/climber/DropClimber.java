/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class DropClimber extends Command {

  Timer m_timer = new Timer();

  public DropClimber() {
    requires(Robot.m_climber);
    requires(Robot.m_driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    m_timer.stop();
    m_timer.reset();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (Robot.m_climber.getFrontClimberHeight() > RobotConstants.FRONT_CLIMBER_GROUND_HEIGHT+0.05) {
      Robot.m_driveTrain.mecanumDrive(0.0, 0.0, 0.0);
    } else {
      Robot.m_driveTrain.mecanumDrive(-0.12, 0.0, 0.0);
    }
    if (Math.abs(Robot.m_climber.getFrontClimberHeight() - RobotConstants.FRONT_CLIMBER_MID_LEVEL_HEIGHT) > 0.3 && m_timer.get() == 0) {
      if (Robot.m_climber.getFrontClimberHeight() > RobotConstants.FRONT_CLIMBER_GROUND_HEIGHT+0.05) { 
        //smaller added constant here^^^^^^????????????????????????
        //?? smaller constant will mean less delay until front legs go down at 0.9 
        //but that may be negligible i want die
        Robot.m_climber.setFront(0.9);
      } else {
        Robot.m_climber.setFront(0.1);
      }
    } else { 
      if (m_timer.get() == 0) {
        Robot.m_climber.setFront(0.0);
        m_timer.start();
      } else if (m_timer.get() > 0.5) {
        Robot.m_climber.setFront(-0.4);
        Robot.m_climber.setBack(-0.4);
      } else {
        Robot.m_climber.setFront(0.0);
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_climber.stopClimber();
    Robot.m_driveTrain.stop();
    m_timer.stop();
    m_timer.reset();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
