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
import frc.robot.commands.drivetrain.SetDriveTrain;
import frc.robot.commands.intake.SetAndRunIntake;
import frc.robot.commands.intakeelbow.SetIntakePos;
/**
 * Makes the climber run with human imput for when to run and for how long
 */
public class LiftRobot extends Command {

  double m_backLiftTime = 0;
  double m_frontStiltTimer = 5;
  Timer m_climbTimer = new Timer();
  SetAndRunIntake m_setIntakeDown = new SetAndRunIntake(RobotConstants.INTAKE_POT_DOWN, 0.2);
  SetDriveTrain m_driveForwards = new SetDriveTrain(0.15);
  SetIntakePos m_liftIntake = new SetIntakePos(RobotConstants.INTAKE_POT_UP, 0.3);

  public LiftRobot() {
    requires(Robot.m_climber);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    if(Robot.m_oi.getOperatorStick().getPOV() == 0) {

      if (m_climbTimer.get() > m_frontStiltTimer && Math.abs(Robot.m_climber.getFrontClimberHeight() - RobotConstants.FRONT_CLIMBER_MIN_HEIGHT) > 0.2) {
        Robot.m_climber.setFront(-0.4);
        if (Robot.m_climber.getBackClimberHeight() > 5.2)
          Robot.m_climber.setBack(0.3975);
        else
          Robot.m_climber.setBack(0.8);
      } else if (m_climbTimer.get() > m_frontStiltTimer) {
        if (m_backLiftTime == 0)
          m_backLiftTime = m_climbTimer.get() + 1.5;
        if (m_climbTimer.get() > m_backLiftTime) {
          if (Math.abs(Robot.m_climber.getBackClimberHeight() - RobotConstants.BACK_CLIMBER_MIN_HEIGHT) > 0.2)
            Robot.m_climber.setBack(-0.4);
          else 
            Robot.m_climber.setBack(0.0);
        } else {
          if (Robot.m_climber.getBackClimberHeight() > 5.2)
            Robot.m_climber.setBack(0.3975);
          else
            Robot.m_climber.setBack(0.8);
        }
        if (!m_driveForwards.isRunning())
          m_driveForwards.start();
        if (!m_liftIntake.isRunning()) {
          m_setIntakeDown.cancel();
          m_liftIntake.start();
        } 
        Robot.m_climber.setFront(0.0);
      } else {
        if (Robot.m_climber.getFrontClimberHeight() > 8)
          Robot.m_climber.setFront(0.45);
        else
          Robot.m_climber.setFront(0.9);

        if (Robot.m_climber.getBackClimberHeight() > 5.2)
          Robot.m_climber.setBack(0.3975);
        else
          Robot.m_climber.setBack(0.8);

          if (!m_setIntakeDown.isRunning()) {
            m_setIntakeDown.start();
            m_climbTimer.reset();
            m_climbTimer.start();
          }
      }

    } else if (Robot.m_oi.getOperatorStick().getPOV() == 90) {

      Robot.m_climber.setFront(-0.4);

      m_setIntakeDown.cancel();
      m_driveForwards.cancel();
      m_liftIntake.cancel();
      m_climbTimer.stop();
      m_climbTimer.reset();

    } else if (Robot.m_oi.getOperatorStick().getPOV() == 180) {

      Robot.m_climber.setFront(-0.375);
      Robot.m_climber.setBack(-0.45);

      m_setIntakeDown.cancel();
      m_driveForwards.cancel();
      m_liftIntake.cancel();
      m_climbTimer.stop();
      m_climbTimer.reset();

    } else if (Robot.m_oi.getOperatorStick().getPOV() == 270) {

      Robot.m_climber.setBack(-0.4);

      m_setIntakeDown.cancel();
      m_driveForwards.cancel();
      m_liftIntake.cancel();
      m_climbTimer.stop();
      m_climbTimer.reset();

    } else {
      Robot.m_climber.setClimber(0);

      m_setIntakeDown.cancel();
      m_driveForwards.cancel();
      m_liftIntake.cancel();
      m_climbTimer.stop();
      m_climbTimer.reset();
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
