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

public class LiftRobotVelocity extends Command {
  double m_velocity;
  public LiftRobotVelocity(double velocity) {
    requires(Robot.m_climber);

    m_velocity = velocity;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (m_velocity < 0) {
      Robot.m_climber.setFrontPIDController(RobotConstants.PID_CLIMBER_FRONT_DOWN_VELOCITY_P, RobotConstants.PID_CLIMBER_FRONT_DOWN_VELOCITY_I, RobotConstants.PID_CLIMBER_FRONT_DOWN_VELOCITY_D);
      Robot.m_climber.setBackPIDController(RobotConstants.PID_CLIMBER_DOWN_VELOCITY_P, RobotConstants.PID_CLIMBER_DOWN_VELOCITY_I, RobotConstants.PID_CLIMBER_DOWN_VELOCITY_D);
    } else {
      Robot.m_climber.setFrontPIDController(RobotConstants.PID_CLIMBER_FRONT_VELOCITY_P, RobotConstants.PID_CLIMBER_FRONT_VELOCITY_I, RobotConstants.PID_CLIMBER_FRONT_VELOCITY_D);
      Robot.m_climber.setBackPIDController(RobotConstants.PID_CLIMBER_VELOCITY_P, RobotConstants.PID_CLIMBER_VELOCITY_I, RobotConstants.PID_CLIMBER_VELOCITY_D);
    }

    Robot.m_climber.getFrontPIDController().setOutputRange(-0.4, 0.4);
    Robot.m_climber.getBackPIDController().setOutputRange(-0.4, 0.4);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_climber.getFrontPIDController().setReference(m_velocity, ControlType.kVelocity);
    Robot.m_climber.getBackPIDController().setReference(m_velocity, ControlType.kVelocity);
    System.out.println("output" + Robot.m_climber.getFrontClimberHeight());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (m_velocity > 0 && Robot.m_climber.isClimberTopped() || (m_velocity < 0 && (Robot.m_climber.getFrontClimberHeight() < 1.5 && Robot.m_climber.getBackClimberHeight() < 1.5)));
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_climber.stopClimber();
  }
}
