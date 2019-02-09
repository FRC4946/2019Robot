/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;


import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class VectorDrive extends Command {

  private double m_verticalSpeed, m_horizontalSpeed;
  

  /**
   * Moves the robot at the desired speed for the desired time. Must be timed out
   * in a command group or with an executor or it will run indefinitely
   *
   * @param verticalSpeed   the speed for the robot to move forwards/backwards at
   *                        as a fraction of its maximum speed
   * @param horizontalSpeed the speed for the robot to move left/right at as a
   *                        fraction of its maximum speed
   */
  public VectorDrive(double verticalSpeed, double horizontalSpeed) {
    requires(Robot.m_driveTrain);

    this.m_horizontalSpeed = horizontalSpeed;
    this.m_verticalSpeed = verticalSpeed;

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_driveTrain.mecanumDrive(m_verticalSpeed, m_horizontalSpeed, 0.0);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
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
