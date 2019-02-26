/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class ClimberDefault extends Command {
  public ClimberDefault() {
    requires(Robot.m_climber);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    /*
    Robot.m_climber.getFrontPIDController().setOutputRange(-0.4, 0.4);
    Robot.m_climber.getBackPIDController().setOutputRange(-0.4, 0.4);
    Robot.m_climber.setFrontPIDController(RobotConstants.PID_CLIMBER_FRONT_POSITION_P, RobotConstants.PID_CLIMBER_FRONT_POSITION_I, RobotConstants.PID_CLIMBER_FRONT_POSITION_D);
    Robot.m_climber.setBackPIDController(RobotConstants.PID_CLIMBER_POSITION_P, RobotConstants.PID_CLIMBER_POSITION_I, RobotConstants.PID_CLIMBER_POSITION_D);
    */



  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if (Robot.m_oi.getOperatorStick().getPOV() == 90) {
      Robot.m_climber.setBack(0.5);
    } else if (Robot.m_oi.getOperatorStick().getPOV() == 270) {
      Robot.m_climber.setFront(-0.5);
    } else {
    /*
    if (Robot.m_climber.getFrontClimberHeight() <= 10.0 && Robot.m_climber.getBackClimberHeight() <= 10.0) {
      Robot.m_climber.getFrontPIDController().setReference(4, ControlType.kPosition);
      Robot.m_climber.getBackPIDController().setReference(4 + RobotConstants.CLIMBER_OFFSET, ControlType.kPosition);
    } else {
      Robot.m_climber.stopClimber();
    }
    */
      Robot.m_climber.stopClimber();
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
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
