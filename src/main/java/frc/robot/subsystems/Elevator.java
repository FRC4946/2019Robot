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
	private CANSparkMax m_motor; 
  	private Solenoid m_brake;
	private AnalogPotentiometer m_analogPot;
	private PIDController m_PIDController;
	  

	private boolean m_isBrake = false;


public Elevator (){

	m_motor = new CANSparkMax(RobotMap.CAN_RUN_ELEVATOR, MotorType.kBrushless);
  	m_brake = new Solenoid(RobotMap.PCM_ELEVATOR_BREAK);
  	m_analogPot = new AnalogPotentiometer(RobotMap.ANALOG_ELEVATOR_POT, 
		RobotConstants.ELEVATOR_SCALING_VALUE, RobotConstants.ELEVATOR_OFFSET_VALUE);
	m_analogPot.setPIDSourceType(PIDSourceType.kDisplacement);
	
	//Set up PID
	m_PIDController = new PIDController(0, 0, 0, m_analogPot, m_motor);
	m_PIDController.setInputRange(RobotConstants.ELEVATOR_MAXIMUM_HEIGHT, RobotConstants.ELEVATOR_MINIMUM_HEIGHT);

	//m_motor.setInverted(true/false); - needed??????
}

  @Override
  public void initDefaultCommand() {
    //setDefaultCommand(command); - if we are doing joystick
  }

  public void enablePID() {
	 //
  }

  public void getHeight(double height) {
	  //return m_analogPot.get();
  }

  public void setPoints(double level) {
	m_PIDController.setSetpoint(level);
  }

  public void setBrake (boolean isBrake){
	m_isBrake = isBrake;
	m_brake.set(!isBrake);
  }

}
