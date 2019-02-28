/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.command.PIDCommand;
import frc.robot.DummyPIDOutput;
import frc.robot.PIDSourceEnc;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class SetClimberToPos extends PIDCommand {

  PIDController m_front, m_back;
  PIDSourceEnc m_frontSource, m_backSource;
  double m_pos;

  public SetClimberToPos(double pos, double speed) {
    super(0.0, 0.0, 0.0);
    requires(Robot.m_climber);
    m_frontSource = new PIDSourceEnc(Robot.m_climber.getFrontEncoder());
    m_backSource = new PIDSourceEnc(Robot.m_climber.getBackEncoder());
    m_pos = pos;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() { 
    m_front = new PIDController(0.2, 0.0, 0.0, m_frontSource, new DummyPIDOutput());
    m_front.setAbsoluteTolerance(2);
    m_front.setContinuous(false);

    m_back = new PIDController(0.1, 0.0, 0.0, m_backSource, new DummyPIDOutput());
    m_back.setAbsoluteTolerance(2);
    m_back.setContinuous(false);

    m_front.setSetpoint(m_pos);
    m_back.setSetpoint(m_pos);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_climber.setClimberFront(m_front.get()*0.8);
    Robot.m_climber.setClimberFront(m_back.get()*0.8);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return m_front.onTarget() && m_back.onTarget();
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

  @Override
  protected double returnPIDInput() {
    return 0.0;
  }

  @Override
  protected void usePIDOutput(double output) {

  }
}
