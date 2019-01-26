/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.Utilities;

public class AbsTurnPID extends PIDCommand {

  private double m_angle, m_maxSpeed; // angle to turn to in degrees

  /**
   * Turns the robot on the spot to the gyro angle provided
   *
   * @param angle the angle to turn to in degrees
   */
  public AbsTurnPID(double angle, double maxSpeed) {

    //DUMMY P I and D values
    super(RobotConstants.CAN_DRIVE_GYRO_TURN_KP,
      RobotConstants.CAN_DRIVE_GYRO_TURN_KI, RobotConstants.CAN_DRIVE_GYRO_TURN_KD);

    requires(Robot.m_driveTrain);

    m_angle = Utilities.conformAngle(angle);
    m_maxSpeed = maxSpeed;

    getPIDController().setInputRange(0.0, 360.0);
    getPIDController().setOutputRange(-0.8, 0.8);
    getPIDController().setContinuous(true);
    getPIDController().setAbsoluteTolerance(2.0);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    getPIDController().enable();
    getPIDController().setSetpoint(m_angle);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return getPIDController().onTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    getPIDController().disable();
    Robot.m_driveTrain.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }

  @Override
  protected double returnPIDInput() {
    return Robot.m_driveTrain.getGyroAngle();
  }

  @Override
  protected void usePIDOutput(double output) {
    Robot.m_driveTrain.mecanumDrive(0.0, 0.0,
      Math.abs(output) > Math.abs(m_maxSpeed) ? m_maxSpeed : output);
  }
}
