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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.DummyPIDOutput;
import frc.robot.PIDSourceEnc;
import frc.robot.PIDSourceGyro;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class LiftRobotPosition extends Command {

  PIDController m_frontPosPID, m_backPosPID; 
  double m_desiredPos, m_maxSpeed, m_desiredPosInit, m_maxSpeedInit;
  double m_frontPosPIDGet, m_backPosPIDGet, m_frontVelPIDGet, m_backVelPIDGet; 
  int m_direction; //1 or -1

  public LiftRobotPosition(double speed) {
    this(Robot.m_climber.getFrontClimberHeight(), speed);
  }

  public LiftRobotPosition(double desiredPos, double maxSpeed) {

    requires(Robot.m_climber);

    m_desiredPos = desiredPos;
    m_maxSpeed = maxSpeed;

    m_frontPosPID = new PIDController(RobotConstants.PID_CLIMBER_FRONT_DOWN_POSITION_P, 
      RobotConstants.PID_CLIMBER_FRONT_DOWN_POSITION_I, 
      RobotConstants.PID_CLIMBER_FRONT_DOWN_POSITION_D,
      Robot.m_climber.getFrontPot(),
      new DummyPIDOutput(), 0.05);

    m_backPosPID = new PIDController(RobotConstants.PID_CLIMBER_DOWN_POSITION_P, 
      RobotConstants.PID_CLIMBER_DOWN_POSITION_I, 
      RobotConstants.PID_CLIMBER_DOWN_POSITION_D,
      Robot.m_climber.getBackPot(),
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

      m_backPosPID.setPID(RobotConstants.PID_CLIMBER_DOWN_POSITION_P, 
        RobotConstants.PID_CLIMBER_DOWN_POSITION_I, 
        RobotConstants.PID_CLIMBER_DOWN_POSITION_D);

      Robot.m_climber.setFrontPIDController(RobotConstants.PID_CLIMBER_FRONT_DOWN_VELOCITY_P, 
        RobotConstants.PID_CLIMBER_FRONT_DOWN_VELOCITY_I, 
        RobotConstants.PID_CLIMBER_FRONT_DOWN_VELOCITY_D);

      Robot.m_climber.setBackPIDController(RobotConstants.PID_CLIMBER_DOWN_VELOCITY_P, 
        RobotConstants.PID_CLIMBER_DOWN_VELOCITY_I, 
        RobotConstants.PID_CLIMBER_DOWN_VELOCITY_D);

    } else {

      m_direction = 1;

      m_frontPosPID.setPID(RobotConstants.PID_CLIMBER_FRONT_POSITION_P, 
        RobotConstants.PID_CLIMBER_FRONT_POSITION_I, 
        RobotConstants.PID_CLIMBER_FRONT_POSITION_D);

      m_backPosPID.setPID(RobotConstants.PID_CLIMBER_POSITION_P, 
        RobotConstants.PID_CLIMBER_POSITION_I, 
        RobotConstants.PID_CLIMBER_POSITION_D);

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
    m_backPosPID.setOutputRange(-m_maxSpeedInit, m_maxSpeedInit); //rpm

    m_frontPosPID.setAbsoluteTolerance(1);
    m_backPosPID.setAbsoluteTolerance(1);

    m_frontPosPID.setContinuous(false);
    m_backPosPID.setContinuous(false);

    m_frontPosPID.setSetpoint(m_desiredPos);
    m_backPosPID.setSetpoint(m_desiredPos);

    m_frontPosPID.enable();
    m_backPosPID.enable();
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

    if(!m_backPosPID.isEnabled()) {
      m_backPosPID.setEnabled(true);
    }

    if(m_direction == 1) {

      if (m_frontHeight < m_desiredPos - 3) {
        m_frontPosPIDGet = m_frontPosPID.get() 
          + m_direction*Math.sqrt(2*(RobotConstants.PID_CLIMBER_FRONT_POSITION_FF + 500*(m_desiredPos - 30))*Math.abs(m_desiredPos - 1 - m_frontHeight));
      } else if(m_desiredPos < m_frontHeight) { 
        m_frontPosPIDGet = 0;
        m_frontPosPID.disable();
      } 

      if(m_desiredPos + 2 < m_backHeight) {
        m_backPosPIDGet = 0;
      } else if (m_backHeight < m_desiredPos + 1) {
        m_backPosPIDGet = m_backPosPID.get() 
        + m_direction*Math.sqrt(2*RobotConstants.PID_CLIMBER_POSITION_FF*Math.abs(m_desiredPos + 2 - m_backHeight));
      }
      
    } else {
      
      if(m_desiredPos + 1 > m_frontHeight) {
        m_frontPosPIDGet = 0;
      } else if (m_frontHeight > m_desiredPos) {
        m_frontPosPIDGet = m_frontPosPID.get() 
        + m_direction*Math.sqrt(2*RobotConstants.PID_CLIMBER_FRONT_POSITION_FF*Math.abs(m_desiredPos - m_frontHeight));
      }

      if(m_desiredPos + 1 > m_backHeight) {
        m_backPosPIDGet = 0;
      } else if (m_backHeight > m_desiredPos) {
        m_backPosPIDGet = m_backPosPID.get() 
        + m_direction*Math.sqrt(2*RobotConstants.PID_CLIMBER_POSITION_FF*Math.abs(m_desiredPos - m_backHeight));
      }
    }

    Robot.m_climber.getFrontPIDController().setReference(m_frontPosPIDGet, ControlType.kVelocity);
    Robot.m_climber.getBackPIDController().setReference(m_backPosPIDGet, ControlType.kVelocity);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return m_frontPosPID.onTarget() && m_backPosPID.onTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_climber.stopClimber();

    m_frontPosPID.reset();
    m_backPosPID.reset();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
