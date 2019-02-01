/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class TimedIntake extends Command {

  private double m_time, m_speed;
  private Timer m_timer = new Timer();

  /**
   * Runs the intake at the desired speed for the desired amount of time
   *
   * @param speed the speed to run the intake at as a fraction of its max speed
   * @param time  the time to run the intake for in seconds
   */
  public TimedIntake(double speed, double time) {
    requires(Robot.m_intake);
    m_time = speed;
    m_time = time;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    m_timer.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_intake.runAll(m_speed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return m_timer.get() >= m_time;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_intake.stopAll();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
