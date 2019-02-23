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
import frc.robot.Utilities;
import frc.robot.commands.grabberarm.SetArmToPos;
import frc.robot.commands.intakeelbow.SetIntakeStage;

public class SetElevatorJoystick extends Command {

  public SetElevatorJoystick() {
    requires(Robot.m_elevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    Robot.m_elevator.setElevator(Utilities.deadzone(-Robot.m_oi.getDriveStick().getRawAxis(1)*1.0));

    if(Robot.m_elevator.getHeight() >= RobotConstants.ELEVATOR_NO_CONFLICT_HEIGHT - 0.1 
      && Robot.m_oi.getDriveStick().getRawAxis(1) > 0) {

      if(Robot.m_grabber.getGrabberIn() 
        && Math.abs(Robot.m_grabberArm.getPos() - RobotConstants.GRABBER_ARM_HOLD_BALL) > 0.1) {

        new SetArmToPos(RobotConstants.GRABBER_ARM_HOLD_BALL, 0.8).start();

      } else if (Robot.m_grabber.getGrabberOut()
        && Math.abs(Robot.m_grabberArm.getPos() - RobotConstants.GRABBER_ARM_HOLD_HATCH) > 0.1) {

        new SetArmToPos(RobotConstants.GRABBER_ARM_HOLD_HATCH, 0.8).start();
      }

      new SetIntakeStage(RobotConstants.INTAKE_POT_UP).start();

    } else if(Robot.m_elevator.getHeight() <= RobotConstants.ELEVATOR_NO_CONFLICT_HEIGHT + 0.1 
      && Robot.m_oi.getDriveStick().getRawAxis(1) < 0) {

      new SetIntakeStage(RobotConstants.INTAKE_POT_BALL_HEIGHT).start();
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
    Robot.m_elevator.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
