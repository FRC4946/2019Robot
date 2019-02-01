/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.Utilities;

public class AbsTurn extends Command {

  private double m_angle; // angle to turn to in degrees
  private boolean m_turnLeft; // whether to turn left or not

  /**
   * Turns the robot on the spot to the gyro angle provided
   *
   * @param angle the angle to turn to in degrees
   */
  public AbsTurn(double angle) {
    requires(Robot.m_driveTrain);
    this.m_angle = Robot.m_utilities.conformAngle(angle);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // turns left if these conditions are both true or both false
    // both true: current angle is 1 degree, angle to turn to is 45
    // both false: current angle is 356 degrees, angle to turn to is 1
    // both cases require turn left
    m_turnLeft = ((m_angle - Robot.m_driveTrain.getGyroAngle()) > 0 == Math
        .abs(m_angle - Robot.m_driveTrain.getGyroAngle()) <= 180);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (m_turnLeft) {
      Robot.m_driveTrain.mecanumDrive(0.0, 0.0, -0.3);
    } else {
      Robot.m_driveTrain.mecanumDrive(0.0, 0.0, 0.3);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Math.abs(Robot.m_driveTrain.getGyroAngle() - m_angle) < 2); // within 2 degrees
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
