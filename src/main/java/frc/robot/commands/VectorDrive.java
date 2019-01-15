/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class VectorDrive extends Command {

  double verticalSpeed, horizontalSpeed, time;
  Timer timer;

  /** 
   *  Moves the robot at the desired speed for the desired time.
   * 
   *  @param verticalSpeed the speed for the robot to move forwards/backwards at as a fraction of its maximum speed
   *  @param horizontalSpeed the speed for the robot to move left/right at as a fraction of its maximum speed
   *  @param time the amount time for the robot to move in seconds
   */
  public VectorDrive(double verticalSpeed, double horizontalSpeed, double time) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_driveTrain);

    this.horizontalSpeed = horizontalSpeed;
    this.verticalSpeed = verticalSpeed;
    this.time = time;

    timer = new Timer();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    timer.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (timer.get() < time) {
        Robot.m_driveTrain.mecanumDrive(verticalSpeed, horizontalSpeed, 0.0);
    } 
    else {
      Robot.m_driveTrain.stop();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return timer.get() >= time;
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
