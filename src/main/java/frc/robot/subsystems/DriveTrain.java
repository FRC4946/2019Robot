/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.RobotMap;
import frc.robot.commands.JoystickDrive;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private CANSparkMax m_leftFront;
  private CANSparkMax m_leftBack;
  private CANSparkMax m_rightFront;
  private CANSparkMax m_rightBack;

  private MecanumDrive m_mecanumDrive;

  public DriveTrain (){

    m_leftFront = new CANSparkMax(RobotMap.CAN_DRIVE_LEFT_FRONT, MotorType.kBrushless);
    m_leftBack = new CANSparkMax(RobotMap.CAN_DRIVE_LEFT_BACK, MotorType.kBrushless);
    m_rightFront = new CANSparkMax(RobotMap.CAN_DRIVE_RIGHT_FRONT, MotorType.kBrushless);
    m_rightBack = new CANSparkMax(RobotMap.CAN_DRIVE_RIGHT_BACK, MotorType.kBrushless);

    m_mecanumDrive = new MecanumDrive(m_leftFront, m_rightFront, m_leftBack, m_rightBack);
  }
  
  @Override
  public void initDefaultCommand() {

    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new JoystickDrive());

  }

  public void mecanumDrive (double Y, double X, double Z){
    m_mecanumDrive.driveCartesian(Y, X, Z);
  }

  public void stop (){
    m_mecanumDrive.driveCartesian(0.0, 0.0, 0.0);
  }

  public void mecanumDriveAbs(double y, double x , double rotation, double gyroAngle) {
    m_mecanumDrive.driveCartesian(y, x, rotation, gyroAngle);
  }
}
