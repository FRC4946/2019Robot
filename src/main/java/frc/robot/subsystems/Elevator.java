/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotConstants;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {

	private CANSparkMax m_elevatorMotor; 
	private AnalogPotentiometer m_analogPot;

	public Elevator (){

		m_elevatorMotor = new CANSparkMax(RobotMap.CAN_RUN_ELEVATOR, MotorType.kBrushless);
  	m_analogPot = new AnalogPotentiometer(RobotMap.ANALOG_ELEVATOR_POT, 
			RobotConstants.ELEVATOR_SCALING_VALUE, RobotConstants.ELEVATOR_OFFSET_VALUE);
		m_analogPot.setPIDSourceType(PIDSourceType.kDisplacement);
	}

  @Override
  public void initDefaultCommand() {

  }

  public double getHeight() {
	  return m_analogPot.get();
  }

  public void setElevator(double speed) {
		m_elevatorMotor.set(speed);
	}

	public CANSparkMax getMotor() {
		return m_elevatorMotor;
	}

	public AnalogPotentiometer getPot() {
		return m_analogPot;
	}
}
