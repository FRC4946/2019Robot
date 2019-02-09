/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.PIDCommand;
import frc.robot.DummyPIDOutput;
import frc.robot.Robot;

public class DriveStraightPID extends PIDCommand {

  double m_dist;
  DummyPIDOutput m_dummyOutput;
  PIDController m_gyroController;

  public DriveStraightPID(double dist) {

    super(0.005875, 0.00002, 0.002); //0.0075

    requires(Robot.m_driveTrain);
    m_dist = dist;
    m_dummyOutput = new DummyPIDOutput();
    m_gyroController = new PIDController(0.00645, 0.000001, 0.002, 
      Robot.m_driveTrain.getGyro(), m_dummyOutput);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    Robot.m_driveTrain.resetEncs();

    getPIDController().setSetpoint(m_dist);
    getPIDController().setOutputRange(-0.7, 0.7);
    getPIDController().setAbsoluteTolerance(2);

    m_gyroController.setInputRange(0, 360);
    m_gyroController.setOutputRange(-0.25, 0.25);
    m_gyroController.setContinuous(true);
    m_gyroController.setSetpoint(Robot.m_driveTrain.getGyroAngle());
    m_gyroController.setAbsoluteTolerance(1);
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
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }

  @Override
  public double returnPIDInput() {
    return Robot.m_driveTrain.getAvgStraightDist();
  }

  @Override
  public void usePIDOutput(double output) {
    Robot.m_driveTrain.mecanumDrive(output, 0.0, m_gyroController.get());
  }
}
