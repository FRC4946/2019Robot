/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.deprecated;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class Vision extends Command {

  private double m_headingErr;
  private double m_distanceErr;
  private double m_steeringAdj;
  private double m_driveAdj;
  private double m_kP;

  public Vision() {
    requires(Robot.m_driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    m_kP = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Periodic Updates To Info
    Robot.m_limelight.m_xOffset = Robot.m_limelight.m_tx.getDouble(0.0); // Coordinate updates
    Robot.m_limelight.m_yOffset = Robot.m_limelight.m_ty.getDouble(0.0);
    Robot.m_limelight.m_area = Robot.m_limelight.m_ta.getDouble(0.0);
    Robot.m_limelight.m_detected = Robot.m_limelight.m_tv.getDouble(0); // see if limelight has detected anytihng
    SmartDashboard.putNumber("LimelightX", Robot.m_limelight.m_xOffset); // Dashboard updates
    SmartDashboard.putNumber("LimelightY", Robot.m_limelight.m_yOffset);
    SmartDashboard.putNumber("LimelightArea", Robot.m_limelight.m_area);

    
    
    // Moving To Target
    /*
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
    */
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
