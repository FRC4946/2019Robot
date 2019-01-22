/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class VectorDrive extends Command {

  private double m_verticalSpeed, m_horizontalSpeed, m_time;
  private Timer m_timer;

  /**
   * Moves the robot at the desired speed for the desired time.
   *
   * @param verticalSpeed   the speed for the robot to move forwards/backwards at
   *                        as a fraction of its maximum speed
   * @param horizontalSpeed the speed for the robot to move left/right at as a
   *                        fraction of its maximum speed
   * @param time            the amount time for the robot to move in seconds
   */
  public VectorDrive(double verticalSpeed, double horizontalSpeed, double time) {
    requires(Robot.m_driveTrain);

    this.m_horizontalSpeed = horizontalSpeed;
    this.m_verticalSpeed = verticalSpeed;
    this.m_time = time;

    m_timer = new Timer();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    m_timer.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (m_timer.get() < m_time) {
      Robot.m_driveTrain.mecanumDrive(m_verticalSpeed, m_horizontalSpeed, 0.0);
    } else {
      Robot.m_driveTrain.stop();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return m_timer.get() >= m_time;
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
