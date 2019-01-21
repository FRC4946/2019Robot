/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class AbsTurnPID extends Command {

  private double m_angle; // angle to turn to in degrees
  private boolean m_turnLeft; // whether to turn left or not
  private PIDController m_turnPID;

  /**
   * Turns the robot on the spot to the gyro angle provided
   *
   * @param angle the angle to turn to in degrees
   */
  public AbsTurnPID(double angle) {
         
    requires(Robot.m_driveTrain);
   
    //DUMMY P I and D values
    m_turnPID = new PIDController(0.1, 0.0, 0.0, (PIDSource) Robot.m_driveTrain.getGyro(), 
      (PIDOutput) new Object());
    this.m_angle = Robot.m_driveTrain.conformAngle(angle);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //turns left if these conditions are both true or both false
    //both true: current angle is 1 degree, angle to turn to is 45
    //both false: current angle is 356 degrees, angle to turn to is 1
    //both cases require turn left
    m_turnPID.enable();
    m_turnPID.setInputRange(0, 360);
    m_turnPID.setOutputRange(-0.8, 0.8);
    m_turnPID.setContinuous(true);
    m_turnPID.setAbsoluteTolerance(2);
    m_turnPID.setSetpoint(m_angle);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_driveTrain.mecanumDrive(0.0, 0.0, m_turnPID.get());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return m_turnPID.onTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    m_turnPID.disable();
    Robot.m_driveTrain.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
