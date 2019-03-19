/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.DummyPIDOutput;
import frc.robot.PIDSourceEnc;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class LiftRobotFrontPos extends Command {

  PIDController m_frontPosPID; 
  double m_desiredPos, m_maxSpeed, m_desiredPosInit, m_maxSpeedInit;
  double m_frontPosPIDGet, m_frontVelPIDGet; 
  int m_direction; //1 or -1

  public LiftRobotFrontPos(double speed) {
    this(Robot.m_climber.getFrontClimberHeight(), speed);
  }

  public LiftRobotFrontPos(double desiredPos, double maxSpeed) {

    requires(Robot.m_climber);

    m_desiredPos = desiredPos;
    m_maxSpeed = maxSpeed;
    
    m_frontPosPID = new PIDController(RobotConstants.PID_CLIMBER_FRONT_DOWN_POSITION_P, 
      RobotConstants.PID_CLIMBER_FRONT_DOWN_POSITION_I, 
      RobotConstants.PID_CLIMBER_FRONT_DOWN_POSITION_D,
      Robot.m_climber.getFrontPot(),
      new DummyPIDOutput(), 0.05);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    m_desiredPosInit = m_desiredPos;
    m_maxSpeedInit = Math.abs(m_maxSpeed);

    if (m_desiredPos < Robot.m_climber.getFrontClimberHeight()) {

      m_direction = -1;

      m_frontPosPID.setPID(RobotConstants.PID_CLIMBER_FRONT_DOWN_POSITION_P, 
        RobotConstants.PID_CLIMBER_FRONT_DOWN_POSITION_I, 
        RobotConstants.PID_CLIMBER_FRONT_DOWN_POSITION_D);

      Robot.m_climber.setFrontPIDController(RobotConstants.PID_CLIMBER_FRONT_DOWN_VELOCITY_P, 
        RobotConstants.PID_CLIMBER_FRONT_DOWN_VELOCITY_I, 
        RobotConstants.PID_CLIMBER_FRONT_DOWN_VELOCITY_D);

    } else {

      m_direction = 1;

      m_frontPosPID.setPID(RobotConstants.PID_CLIMBER_FRONT_POSITION_P, 
        RobotConstants.PID_CLIMBER_FRONT_POSITION_I, 
        RobotConstants.PID_CLIMBER_FRONT_POSITION_D);

      Robot.m_climber.setFrontPIDController(RobotConstants.PID_CLIMBER_FRONT_VELOCITY_P, 
        RobotConstants.PID_CLIMBER_FRONT_VELOCITY_I, 
        RobotConstants.PID_CLIMBER_FRONT_VELOCITY_D);

      Robot.m_climber.setBackPIDController(RobotConstants.PID_CLIMBER_VELOCITY_P, 
        RobotConstants.PID_CLIMBER_VELOCITY_I, 
        RobotConstants.PID_CLIMBER_VELOCITY_D);
    }
    
    Robot.m_climber.getBackPIDController().setOutputRange(-0.4, 0.4);
    Robot.m_climber.getFrontPIDController().setOutputRange(-0.4, 0.4);

    m_frontPosPID.setOutputRange(-m_maxSpeedInit, m_maxSpeedInit); //rpm - there exists a max rpm that is a cap

    m_frontPosPID.setAbsoluteTolerance(1);

    m_frontPosPID.setContinuous(false);

    m_frontPosPID.setSetpoint(m_desiredPos);

    m_frontPosPID.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    double m_frontHeight, m_backHeight;

    m_frontHeight = Robot.m_climber.getFrontClimberHeight();
    m_backHeight = Robot.m_climber.getBackClimberHeight();

    if(!m_frontPosPID.isEnabled()) {
      m_frontPosPID.setEnabled(true);
    }

    if(m_direction == 1) {

      if (m_frontHeight < m_desiredPos - 3) {
        m_frontPosPIDGet = m_frontPosPID.get() 
          + m_direction*Math.sqrt(2*(RobotConstants.PID_CLIMBER_FRONT_POSITION_FF + 500*(m_desiredPos - 30))*Math.abs(m_desiredPos - 1 - m_frontHeight));
      } else if(m_desiredPos < m_frontHeight) { 
        m_frontPosPIDGet = 0;
        m_frontPosPID.disable();
      } 

    } else {
      
      if(m_desiredPos + 1 > m_frontHeight) {
        m_frontPosPIDGet = 0;
      } else if (m_frontHeight > m_desiredPos) {
        m_frontPosPIDGet = m_frontPosPID.get() 
        + m_direction*Math.sqrt(2*RobotConstants.PID_CLIMBER_FRONT_POSITION_FF*Math.abs(m_desiredPos - m_frontHeight));
      }
    } 

    Robot.m_climber.getFrontPIDController().setReference(m_frontPosPIDGet, ControlType.kVelocity);
   
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return m_frontPosPID.onTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_climber.stopClimber();

    m_frontPosPID.reset();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}