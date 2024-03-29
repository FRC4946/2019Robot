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
import frc.robot.commands.intakeelbow.SetIntakeStage;

public class SetElevator extends Command {
/**
 * Sets the elevator 
 */

  private boolean isBelowConflict;
  private double m_speed;

  public SetElevator(double speed) {
    requires(Robot.m_elevator);
    isBelowConflict = Robot.m_elevator.getHeight() < RobotConstants.ELEVATOR_NO_CONFLICT_HEIGHT;
    m_speed = speed;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    Robot.m_elevator.setElevator(m_speed);
      
    if(isBelowConflict != Robot.m_elevator.getHeight() < RobotConstants.ELEVATOR_NO_CONFLICT_HEIGHT) {
     
      if(Robot.m_elevator.getHeight() >= RobotConstants.ELEVATOR_NO_CONFLICT_HEIGHT) {

        if(Math.abs(Robot.m_grabberArm.getPos() - RobotConstants.GRABBER_ARM_HOLD_HATCH) > 0.2) {
          new SetArmToPos(RobotConstants.GRABBER_ARM_HOLD_HATCH, 0.8).start();
        }

        if(Math.abs(Robot.m_intakeElbow.getPos() - RobotConstants.INTAKE_POT_UP) > 5) {
          new SetIntakePos(RobotConstants.INTAKE_POT_UP, 0.3).start(); 
        } 

      } else if (Robot.m_elevator.getHeight() < RobotConstants.ELEVATOR_NO_CONFLICT_HEIGHT) {

        if(Math.abs(Robot.m_intakeElbow.getPos() - RobotConstants.INTAKE_POT_BALL_HEIGHT) > 5) {
          new SetIntakePos(RobotConstants.INTAKE_POT_BALL_HEIGHT, 0.3).start(); 
        } 

        if(Math.abs(Robot.m_grabberArm.getPos() - RobotConstants.GRABBER_ARM_HOLD_BALL) > 0.2) {
          new SetArmToPos(RobotConstants.GRABBER_ARM_HOLD_BALL, 0.8).start();
        }
      }

      isBelowConflict = !isBelowConflict;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.m_elevator.getHeight() <= RobotConstants.ELEVATOR_AT_BOTTOM && m_speed < 0
    || Robot.m_elevator.getHeight() >= RobotConstants.ELEVATOR_AT_TOP && m_speed > 0;
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
