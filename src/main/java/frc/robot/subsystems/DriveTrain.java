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

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotMap;
import frc.robot.commands.JoystickDrive;

/**
 * TODO: Document
 */
public class DriveTrain extends Subsystem {

  private CANSparkMax m_leftFront, m_leftBack, m_rightFront, m_rightBack;
  private Encoder m_leftFrontEnc, m_leftBackEnc, m_rightFrontEnc, m_rightBackEnc;
  private PIDController m_leftFrontPID, m_leftBackPID, m_rightFrontPID, m_rightBackPID, m_gyroPID;
  private MecanumDrive m_mecanumDrive;
  private AHRS m_gyro;

  private boolean pidEnabled;

  public DriveTrain() {

    m_leftFront = new CANSparkMax(RobotMap.CAN_DRIVE_LEFT_FRONT, MotorType.kBrushless);
    m_leftBack = new CANSparkMax(RobotMap.CAN_DRIVE_LEFT_BACK, MotorType.kBrushless);
    m_rightFront = new CANSparkMax(RobotMap.CAN_DRIVE_RIGHT_FRONT, MotorType.kBrushless);
    m_rightBack = new CANSparkMax(RobotMap.CAN_DRIVE_RIGHT_BACK, MotorType.kBrushless);

    m_leftFrontEnc = new Encoder(RobotMap.CAN_DRIVE_LEFT_FRONT_ENCA, RobotMap.CAN_DRIVE_LEFT_FRONT_ENCB);
    m_leftBackEnc = new Encoder(RobotMap.CAN_DRIVE_LEFT_BACK_ENCA, RobotMap.CAN_DRIVE_LEFT_BACK_ENCB);
    m_rightFrontEnc = new Encoder(RobotMap.CAN_DRIVE_RIGHT_FRONT_ENCA, RobotMap.CAN_DRIVE_RIGHT_FRONT_ENCB);
    m_rightBackEnc = new Encoder(RobotMap.CAN_DRIVE_RIGHT_BACK_ENCA, RobotMap.CAN_DRIVE_RIGHT_BACK_ENCB);

    m_mecanumDrive = new MecanumDrive(m_leftFront, m_rightFront, m_leftBack, m_rightBack);
    m_gyro = new AHRS(Port.kMXP);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new JoystickDrive());
  }

  protected double deadzone(double input) {
    return Math.abs(input) < RobotConstants.JOYSTICK_DEADZONE ? 0 : input;
  }

  /**
   * Drives on the desired vector while turning at the set speed
   *
   * @param Y The forwards speed for the robot
   * @param X The sideways speed for the robot
   * @param Z The rate of rotation for the robot
   */
  public void mecanumDrive(double Y, double X, double Z) {
    m_mecanumDrive.driveCartesian(deadzone(Y), deadzone(X), deadzone(Z));
  }

  public void stop() {
    m_mecanumDrive.driveCartesian(0.0, 0.0, 0.0);
  }

   /**
   * Drives on the desired vector relative to the field while turning at the set speed
   *
   * @param Y The forwards speed for the robot
   * @param X The sideways speed for the robot
   * @param Z The rate of rotation for the robot
   */
  public void mecanumDriveAbs(double Y, double X, double Z) {
    m_mecanumDrive.driveCartesian(deadzone(Y), deadzone(X), deadzone(Z), getGyroAngle());
  }
  
  public double getGyroAngle() {
    return Robot.m_utility.conformAngle(m_gyro.getAngle() % 360.0);
  }

  public AHRS getGyro() {
    return m_gyro;
  }
}
