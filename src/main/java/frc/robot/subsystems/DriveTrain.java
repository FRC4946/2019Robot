/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.RobotMap;
import frc.robot.commands.JoystickDrive;

/**
 * TODO: Document
 */
public class DriveTrain extends Subsystem {

  private CANSparkMax m_leftFront;
  private CANSparkMax m_leftBack;
  private CANSparkMax m_rightFront;
  private CANSparkMax m_rightBack;

  private MecanumDrive m_mecanumDrive;

  private AHRS m_gyro;

  public DriveTrain() {

    m_leftFront = new CANSparkMax(RobotMap.CAN_DRIVE_LEFT_FRONT, MotorType.kBrushless);
    m_leftBack = new CANSparkMax(RobotMap.CAN_DRIVE_LEFT_BACK, MotorType.kBrushless);
    m_rightFront = new CANSparkMax(RobotMap.CAN_DRIVE_RIGHT_FRONT, MotorType.kBrushless);
    m_rightBack = new CANSparkMax(RobotMap.CAN_DRIVE_RIGHT_BACK, MotorType.kBrushless);

    m_mecanumDrive = new MecanumDrive(m_leftFront, m_leftBack, m_rightFront, m_rightBack);
    m_gyro = new AHRS(Port.kMXP);

  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new JoystickDrive());
  }

  /**
   * Drives on the desired vector while turning at the set speed
   *
   * @param Y The forwards speed for the robot
   * @param X The sideways speed for the robot
   * @param Z The rate of rotation for the robot
   */
  public void mecanumDrive(double Y, double X, double Z) {
    m_mecanumDrive.driveCartesian(Y, X, Z);
  }

  public void stop() {
    m_mecanumDrive.driveCartesian(0.0, 0.0, 0.0);
  }

  public void mecanumDriveAbs(double y, double x, double rotation, double gyroAngle) {
    m_mecanumDrive.driveCartesian(y, x, rotation, gyroAngle);
  }

  public double getGyroAngle() {
    if (m_gyro.getAngle() % 360.0 >= 0) {
      return m_gyro.getAngle() % 360.0;
    } else {
      return m_gyro.getAngle() % 360.0 + 360.0;
    }
  }
}
