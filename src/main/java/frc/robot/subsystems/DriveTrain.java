/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.RobotConstants;
import frc.robot.RobotMap;
import frc.robot.Utilities;
import frc.robot.commands.drivetrain.JoystickDrive;

/**
 * TODO: Document
 */
public class DriveTrain extends Subsystem {

  private CANSparkMax m_leftFront, m_leftBack, m_rightFront, m_rightBack;
  private Encoder m_leftFrontEnc, m_leftBackEnc, m_rightFrontEnc, m_rightBackEnc;
  private MecanumDrive m_mecanumDrive;
  private AHRS m_gyro;

  public DriveTrain() {

    m_leftFront = new CANSparkMax(RobotMap.CAN_SPARK_DRIVE_LEFT_FRONT, MotorType.kBrushless);
    m_leftBack = new CANSparkMax(RobotMap.CAN_SPARK_DRIVE_LEFT_BACK, MotorType.kBrushless);
    m_rightFront = new CANSparkMax(RobotMap.CAN_SPARK_DRIVE_RIGHT_FRONT, MotorType.kBrushless);
    m_rightBack = new CANSparkMax(RobotMap.CAN_SPARK_DRIVE_RIGHT_BACK, MotorType.kBrushless);

    m_leftFrontEnc = new Encoder(RobotMap.DIO_DRIVE_LEFT_FRONT_ENCA, RobotMap.DIO_DRIVE_LEFT_FRONT_ENCB);
    m_leftBackEnc = new Encoder(RobotMap.DIO_DRIVE_LEFT_BACK_ENCA, RobotMap.DIO_DRIVE_LEFT_BACK_ENCB);
    m_rightFrontEnc = new Encoder(RobotMap.DIO_DRIVE_RIGHT_FRONT_ENCA, RobotMap.DIO_DRIVE_RIGHT_FRONT_ENCB);
    m_rightBackEnc = new Encoder(RobotMap.DIO_DRIVE_RIGHT_BACK_ENCA, RobotMap.DIO_DRIVE_RIGHT_BACK_ENCB);

    m_leftFrontEnc.setDistancePerPulse(RobotConstants.ENC_DIST_PER_PULSE);
    m_leftBackEnc.setDistancePerPulse(RobotConstants.ENC_DIST_PER_PULSE);
    m_rightFrontEnc.setDistancePerPulse(RobotConstants.ENC_DIST_PER_PULSE);
    m_rightBackEnc.setDistancePerPulse(RobotConstants.ENC_DIST_PER_PULSE);

    m_mecanumDrive = new MecanumDrive(m_leftFront, m_rightFront, m_leftBack, m_rightBack);
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

  /**
   * Drives on the desired vector relative to the field while turning at the set
   * speed
   *
   * @param Y The forwards speed for the robot
   * @param X The sideways speed for the robot
   * @param Z The rate of rotation for the robot
   */
  public void mecanumDriveAbs(double Y, double X, double Z) {
    m_mecanumDrive.driveCartesian(Y, X, Z);
  }

  /**
   * Resets the encoder value resulting in a value of 0
   */
  public void resetEncs() {
    m_leftFrontEnc.reset();
    m_rightFrontEnc.reset();
    m_leftBackEnc.reset();
    m_rightBackEnc.reset();
  }

  /**
   * Gets the angle of the robot using the gyro
   */
  public double getGyroAngle() {
    return Utilities.conformAngle(m_gyro.getAngle());
  }
  /**
   * 
   */
  public double getAvgStraightDist() {
    return (m_leftFrontEnc.getDistance() + m_rightFrontEnc.getDistance() + m_leftBackEnc.getDistance() + m_rightBackEnc.getDistance())/4.0;
  }

  public AHRS getGyro() {
    return m_gyro;
  }

  /**
   * Gets the speed that the front left motor is currently running at
   * @return the speed of the front left motor
   */
  public CANSparkMax getLeftFront() {
    return m_leftFront;
  }

  /**
   * Gets the speed that the front right motor is currently running at
   * @return the speed of the front right motor
   */
  public CANSparkMax getRightFront() {
    return m_rightFront;
  }

  /**
   * Gets the speed that the back left motor is currently running at
   * @return the speed of the back left motor
   */
  public CANSparkMax getLeftBack() {
    return m_leftBack;
  }

   /**
   * Gets the speed that the back right motor is currently running at
   * @return the speed of the back right motor
   */
  public CANSparkMax getRightBack() {
    return m_rightBack;
  }
  /**
   * Gets the value of front left encoder
   * @return the value of the front left encoder
   */
  public Encoder getLeftFrontEnc() {
    return m_leftFrontEnc;
  }

  /**
   * Gets the value of front right encoder
   * @return the value of the front right encoder
   */
  public Encoder getRightFrontEnc() {
    return m_rightFrontEnc;
  }

  /**
   * Gets the value of back left encoder
   * @return the value of the back left encoder
   */
  public Encoder getLeftBackEnc() {
    return m_leftBackEnc;
  }

  /**
   * Gets the value of back right encoder
   * @return the value of the back right encoder
   */
  public Encoder getRightBackEnc() {
    return m_rightBackEnc;
  }

  /**
   * Sets the gains on the designated PID controller
   * @param controller the controller to modify
   * @param p the proportional gain
   * @param i the intergral gain
   * @param d the derivative gain
   */
  public void setPID(CANPIDController controller, double p, double i, double d) {
    controller.setP(p);
    controller.setI(i);
    controller.setD(d);
  }

  /**
   * Assigns a setpoint to a pid controller
   * @param controller the controller to assign a setpoint to
   * @param setpoint a value between -1 and 1 to set as the setpoint
   */
  public void setPIDSetpoint(CANSparkMax controller, double setpoint) {
    controller.pidWrite(setpoint);
  }
}
