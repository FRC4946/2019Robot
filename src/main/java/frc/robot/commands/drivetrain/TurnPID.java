/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.Utilities;

public class TurnPID extends PIDCommand {

double m_angle, m_startAngle;

  public TurnPID(double angle) {
    super(0.00645, 0.000001, 0.002);
    requires(Robot.m_driveTrain);
    m_angle = angle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    m_startAngle = Robot.m_driveTrain.getGyroAngle();

    getPIDController().setInputRange(0, 360);
    getPIDController().setOutputRange(-0.5, 0.5);
    getPIDController().setContinuous(true);
    getPIDController().setAbsoluteTolerance(1);
    getPIDController().setSetpoint(Utilities.conformAngle(m_startAngle + m_angle));
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
    Robot.m_driveTrain.stop();
    Robot.m_driveTrain.resetEncs();
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
    Robot.m_driveTrain.mecanumDrive(0.0, 0.0, output);
  }
}
