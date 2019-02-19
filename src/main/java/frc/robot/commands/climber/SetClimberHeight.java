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

public class SetClimberHeight extends Command {
  
  double m_height;

  public SetClimberHeight(double height) {
    requires(Robot.m_climber);
    m_height = height;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_climber.setFrontPIDController(RobotConstants.PID_CLIMBER_FRONT_POSITION_P, RobotConstants.PID_CLIMBER_FRONT_POSITION_I, RobotConstants.PID_CLIMBER_FRONT_POSITION_D);
    Robot.m_climber.setBackPIDController(RobotConstants.PID_CLIMBER_POSITION_P, RobotConstants.PID_CLIMBER_POSITION_I, RobotConstants.PID_CLIMBER_POSITION_D);
    Robot.m_climber.getFrontPIDController().setOutputRange(-0.4, 0.4);
    Robot.m_climber.getBackPIDController().setOutputRange(-0.2, 0.2);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_climber.getFrontPIDController().setReference(m_height, ControlType.kPosition);
    Robot.m_climber.getBackPIDController().setReference(m_height + RobotConstants.CLIMBER_OFFSET, ControlType.kPosition);
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
