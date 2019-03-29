/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.commands.grabberarm.SetArmToPos;
import frc.robot.commands.intakeelbow.SetIntakePos;

//OLD VERSION IS IN COMMENTS BELOW

public class MoveToHeight extends Command {
  
  double m_height, m_speed, m_speedInit; 
  boolean m_setIntakeToBallIsRunning, m_setIntakeUpIsRunning;
  SetArmToPos m_setOut = new SetArmToPos(RobotConstants.GRABBER_ARM_OUT, 0.7);
  SetArmToPos m_setBall = new SetArmToPos(RobotConstants.GRABBER_ARM_HOLD_BALL, 0.7);
  SetIntakePos m_setIntakeToBall = new SetIntakePos(RobotConstants.INTAKE_POT_BALL_HEIGHT, 0.25);
  SetIntakePos m_setIntakeUp = new SetIntakePos(RobotConstants.INTAKE_POT_UP, 0.2);
  boolean m_isElbowSafetyOn; 

  public MoveToHeight(double height, double speed) {
    this(height, speed, true);
  }

  //the elbowSafetyOn parameter being FALSE will only ever occur in the MoveToLowHeight.java class
  public MoveToHeight(double height, double speed, boolean elbowSafetyOn) {
    requires(Robot.m_elevator);

    m_height = height;
    m_speed = Math.min(Math.abs(speed), 0.9);
    m_isElbowSafetyOn = elbowSafetyOn;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    m_speedInit = m_speed;
    m_speedInit *= Math.signum(m_height - Robot.m_elevator.getHeight());

    m_setIntakeToBallIsRunning = false;
    m_setIntakeUpIsRunning = false;

    if(m_speedInit < 0) {
      m_speedInit *= 0.75;
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() { 

    Robot.m_elevator.setElevator(m_speedInit);

    if (Robot.m_elevator.getHeight() > RobotConstants.ELEVATOR_THRESHOLD) {
      if (Math.abs(Robot.m_grabberArm.getPos() - RobotConstants.GRABBER_ARM_OUT) > 0.2) {
        //m_setBall.cancel();
        m_setOut.start();
      }
    } else if (Robot.m_elevator.getHeight() <= RobotConstants.ELEVATOR_THRESHOLD - 0.01) {
      if (Math.abs(Robot.m_grabberArm.getPos() - RobotConstants.GRABBER_ARM_HOLD_BALL) > 1.0) {
        //m_setOut.cancel();
        m_setBall.start();
      } 
    }

    if(m_isElbowSafetyOn) {

      if(Robot.m_elevator.getHeight() < RobotConstants.ELEVATOR_RIGHT_ABOVE_ELBOW && m_speedInit > 0) {
        
        if(!m_setIntakeToBall.isRunning() && !m_setIntakeToBallIsRunning) {
          m_setIntakeToBall.start();
          m_setIntakeToBallIsRunning = true;
        } 

      } else if (Robot.m_elevator.getHeight() > (RobotConstants.ELEVATOR_RIGHT_ABOVE_ELBOW + 0.01) 
        && m_speedInit > 0) {

        if(!m_setIntakeUp.isRunning() && !m_setIntakeUpIsRunning) {
          m_setIntakeUp.start();
          m_setIntakeUpIsRunning = true;
        }
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (m_speedInit < 0) {
      return Robot.m_elevator.getHeight() <= (m_height + 0.02);
    } else {
      return Robot.m_elevator.getHeight() >= (m_height - 0.02);
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_elevator.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run

  @Override
  protected void interrupted() {
    end();
  }
}

/** 
package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.commands.grabberarm.SetArmToPos;
import frc.robot.commands.intakeelbow.SetIntakePos;

public class MoveToHeight extends Command {
  
  double m_height, m_speed, m_speedInit; 
  boolean m_setIntakeToBallIsRunning, m_setIntakeUpIsRunning;
  SetArmToPos m_setOut = new SetArmToPos(RobotConstants.GRABBER_ARM_OUT, 0.7);
  SetArmToPos m_setBall = new SetArmToPos(RobotConstants.GRABBER_ARM_HOLD_BALL, 0.7);
  SetIntakePos m_setIntakeToBall = new SetIntakePos(RobotConstants.INTAKE_POT_BALL_HEIGHT, 0.25);
  SetIntakePos m_setIntakeUp = new SetIntakePos(RobotConstants.INTAKE_POT_UP, 0.2);

  public MoveToHeight(double height, double speed) {
    requires(Robot.m_elevator);

    m_height = height;
    m_speed = Math.min(Math.abs(speed), 0.9);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    m_speedInit = m_speed;
    m_speedInit *= Math.signum(m_height - Robot.m_elevator.getHeight());

    m_setIntakeToBallIsRunning = false;
    m_setIntakeUpIsRunning = false;

    if(m_speedInit < 0) {
      m_speedInit *= 0.75;
    }
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() { 

    Robot.m_elevator.setElevator(m_speedInit);

    if (Robot.m_elevator.getHeight() > RobotConstants.ELEVATOR_THRESHOLD) {
      if (Math.abs(Robot.m_grabberArm.getPos() - RobotConstants.GRABBER_ARM_OUT) > 0.2) {
        //m_setBall.cancel();
        m_setOut.start();
      }
    } else if (Robot.m_elevator.getHeight() <= RobotConstants.ELEVATOR_THRESHOLD - 0.01) {
      if (Math.abs(Robot.m_grabberArm.getPos() - RobotConstants.GRABBER_ARM_HOLD_BALL) > 1.0) {
        //m_setOut.cancel();
        m_setBall.start();
      } 
    }

    
    if(Robot.m_elevator.getHeight() < RobotConstants.ELEVATOR_RIGHT_ABOVE_ELBOW && m_speedInit > 0) {
      
      if(!m_setIntakeToBall.isRunning() && !m_setIntakeToBallIsRunning) {
        m_setIntakeToBall.start();
        m_setIntakeToBallIsRunning = true;
      } 

    } else if (Robot.m_elevator.getHeight() > (RobotConstants.ELEVATOR_RIGHT_ABOVE_ELBOW + 0.01) 
      && m_speedInit > 0) {

      if(!m_setIntakeUp.isRunning() && !m_setIntakeUpIsRunning) {
        m_setIntakeUp.start();
        m_setIntakeUpIsRunning = true;
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (m_speedInit < 0) {
      return Robot.m_elevator.getHeight() <= (m_height + 0.02);
    } else {
      return Robot.m_elevator.getHeight() >= (m_height - 0.02);
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_elevator.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run

  @Override
  protected void interrupted() {
    end();
  }
}
*/