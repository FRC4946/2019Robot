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
import frc.robot.commands.elevator.SetElevatorJoystick;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {

  private CANSparkMax m_elevatorMotor; 
  private AnalogPotentiometer m_analogPot;

  public Elevator (){

    m_elevatorMotor = new CANSparkMax(RobotMap.CAN_SPARK_ELEVATOR, MotorType.kBrushless);
    m_analogPot = new AnalogPotentiometer(RobotMap.ANALOG_ELEVATOR_POT, 
      RobotConstants.ELEVATOR_SCALING_VALUE, RobotConstants.ELEVATOR_OFFSET_VALUE);
    m_analogPot.setPIDSourceType(PIDSourceType.kDisplacement);
  }

  @Override
  public void initDefaultCommand() { 
    setDefaultCommand(new SetElevatorJoystick());
  }

  /**
   * Gets the height of the elevator
   * @return the analog potentiometer's value
   */
  public double getHeight() {
    //return RobotConstants.ELEVATOR_AT_TOP - m_analogPot.get();
    return m_analogPot.get();
  }

  /**
   * Sets the speed of the elevator's motor to the desired speed
   * @param speed The speed that elevator motor runs at
   */
  public void setElevator(double speed) {
    
    if(speed < 0) {
      speed *= 0.75;
    }

    if (speed < 0 && getHeight() <= RobotConstants.ELEVATOR_AT_MIN + 0.05) {
      stop();
    } else if (speed > 0 && getHeight() >= RobotConstants.ELEVATOR_AT_MAX) {
      stop();
    } else if (Math.abs(getHeight() - RobotConstants.ELEVATOR_AT_BOTTOM) <= 0.5) {
      m_elevatorMotor.set(Math.min(0.4, speed));
    } else {
      m_elevatorMotor.set(speed);
    }
  }

  /**
   * Stops the elevator
   */
  public void stop() {
    setElevator(0);
  }

	/**
 	* Gets the speed that the motor is running at
 	* @return the speed of the motor
 	*/
	public CANSparkMax getMotor() {
		return m_elevatorMotor;
	}

	/**
	 * Gets the value of the analog potentiometer
	 */
	public AnalogPotentiometer getPot() {
		return m_analogPot;
	}
}
