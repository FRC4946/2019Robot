/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class SlowAccel extends Command {

  double m_dist, m_speed, m_time, m_accel, m_currSpeed, m_count;

  public SlowAccel(double dist, double speed) {
    m_dist = dist; m_speed = speed*(5900/60.0)*Math.PI*6.0; 
    requires(Robot.m_driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    m_time = Math.abs(m_dist/(m_speed));
    m_accel = Math.abs(4*Math.pow(m_speed, 2)/m_dist);
    m_currSpeed = 0.0; m_count = 0.0;
    Robot.m_driveTrain.resetEncs();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if(m_count*0.02 < 0.75*m_time) {
      m_currSpeed = Math.abs(m_currSpeed) >= Math.abs(m_speed) ? m_speed : m_currSpeed + m_accel*0.02*Math.signum(m_dist);
    } else {
      m_currSpeed = Math.abs(m_currSpeed) > 0 ? m_currSpeed - m_accel*0.02*Math.signum(m_dist) : 0;
    }

    Robot.m_driveTrain.mecanumDrive(m_currSpeed*60.0/(double)(5900.0*Math.PI*6)*Math.signum(m_dist), 0.0, 0.0);
    m_count++;
    System.out.println("Current Speed: " + m_currSpeed);
    System.out.println("Desired Speed: " + m_speed);
    System.out.println("Acceleration: " + m_accel);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    System.out.println("Ending");
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_driveTrain.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
