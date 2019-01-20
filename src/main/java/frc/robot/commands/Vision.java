/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision extends Command {

  private double m_headingErr;
  private double m_distanceErr;
  private double m_steeringAdj;
  private double m_driveAdj;

  public Vision() {
    requires(Robot.m_driveTrain);
    //requires(Robot.m_limelight);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    m_headingErr = m_headingErr;
    m_distanceErr = m_distanceErr;
    m_steeringAdj = m_steeringAdj;
    m_driveAdj = m_driveAdj;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Periodic Updates To Info
    Robot.m_limelight.xOffset = Robot.m_limelight.tx.getDouble(0.0); // Coordinate updates
    Robot.m_limelight.yOffset = Robot.m_limelight.ty.getDouble(0.0);
    Robot.m_limelight.area = Robot.m_limelight.ta.getDouble(0.0);
    Robot.m_limelight.detected = Robot.m_limelight.tv.getDouble(0); // see if limelight has detected anytihng
    SmartDashboard.putNumber("LimelightX", Robot.m_limelight.xOffset); // Dashboard updates
    SmartDashboard.putNumber("LimelightY", Robot.m_limelight.yOffset);
    SmartDashboard.putNumber("LimelightArea", Robot.m_limelight.area);

    if (Robot.m_oi.getDriveStick().getRawButton(/* Button Number */1) == true) {
      // Moving To Target

      if (Robot.m_limelight.detected == 1.0) {
        m_headingErr = -Robot.m_limelight.xOffset;
        m_distanceErr = -Robot.m_limelight.yOffset;
        m_steeringAdj = 0.0;

        if (Robot.m_limelight.xOffset > 1.0) {
          m_steeringAdj = RobotConstants.LIMELIGHT_TURN_KP * m_headingErr - RobotConstants.MIN_AIM_COMMAND;
        } else if (Robot.m_limelight.xOffset < 1.0) {
          m_steeringAdj = RobotConstants.LIMELIGHT_TURN_KP * m_headingErr + RobotConstants.MIN_AIM_COMMAND;
        }

        m_driveAdj = RobotConstants.LIMELIGHT_DISTANCE_KP * (-1 * Robot.m_limelight.findDistance());

        // Driving
        Robot.m_driveTrain.mecanumDrive(m_steeringAdj + m_driveAdj, 0.0, 0.0);

      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
