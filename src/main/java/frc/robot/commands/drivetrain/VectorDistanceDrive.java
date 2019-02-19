/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.PIDCommand;
import frc.robot.Robot;

public class VectorDistanceDrive extends PIDCommand {
  double[] m_distanceVector, m_unitVector, m_fractionalSpeeds, m_rpmSpeeds, m_inchSpeeds, m_velocityVector, m_wheelDistances;
  double m_distance, m_velocityMag, m_scaleValue;
  public VectorDistanceDrive(double y, double x) {
    super(0.2, 0, 0);
    requires(Robot.m_driveTrain);

    m_distanceVector = new double[] {x, y};
    m_distance = Math.sqrt(Math.pow(m_distanceVector[0] , 2) + Math.pow(m_distanceVector[1], 2));
    m_unitVector = new double[] {m_distanceVector[0]/m_distance, m_distanceVector[1]/m_distance};
    m_scaleValue = m_distanceVector[0] + m_distanceVector[1];

    m_fractionalSpeeds = new double[] {
     (m_distanceVector[1] - m_distanceVector[0])/m_scaleValue,    //front   right 
      (m_distanceVector[1] + m_distanceVector[0])/m_scaleValue,   //front   left
      (m_distanceVector[1] + m_distanceVector[0])/m_scaleValue,   //back    right
      (m_distanceVector[1] - m_distanceVector[0])/m_scaleValue    //back    left
    };

    m_rpmSpeeds = new double[m_fractionalSpeeds.length];
    m_inchSpeeds = new double[m_rpmSpeeds.length];

    for (int i = 0; i < m_fractionalSpeeds.length; i++) {
      m_rpmSpeeds[i] = m_fractionalSpeeds[i] * (5900/6);  //5900/6 is the max speed of the wheels
    }

    for (int i = 0; i < m_rpmSpeeds.length; i++) {
      m_inchSpeeds[i] = (m_rpmSpeeds[i]/60)*(6*Math.PI);  //60 seconds in a minute, 6*math.pi is circumfrence
    }

    m_velocityVector = new double[] {
      (m_inchSpeeds[1] + m_inchSpeeds[3])/2,
      (m_inchSpeeds[1] + m_inchSpeeds[3])/2 - m_inchSpeeds[3]
    };

    m_velocityMag = Math.sqrt(Math.pow(m_velocityVector[0] , 2) + Math.pow(m_velocityVector[1], 2));
    m_wheelDistances = new double[4];

    for (int i = 0; i < m_inchSpeeds.length; i++) {
      m_wheelDistances[i] = (m_distance/m_velocityMag)*m_inchSpeeds[i];
    }

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    getPIDController().setOutputRange(-1.0, 1.0);
    getPIDController().setContinuous(false);
    getPIDController().setAbsoluteTolerance(2);
    getPIDController().setSetpoint(0); //average error must equal zero

    Robot.m_driveTrain.resetEncs();
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
  protected double returnPIDInput() {
    return (
      (m_wheelDistances[0] - Robot.m_driveTrain.getRightFrontEncDistance()) +
      (m_wheelDistances[1] - Robot.m_driveTrain.getLeftFrontEncDistance()) +
      (m_wheelDistances[2] - Robot.m_driveTrain.getRightBackEncDistance()) +
      (m_wheelDistances[3] - Robot.m_driveTrain.getLeftBackEncDistance())
    )/4;
  }

  @Override
  protected void usePIDOutput(double output) {
    Robot.m_driveTrain.setPIDSetpoints(output, m_fractionalSpeeds);
  }
}