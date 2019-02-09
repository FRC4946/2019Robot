*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

 package frc.robot.commands.intake;

 import com.ctre.phoenix.motorcontrol.ControlMode;

 import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.RobotConstants;

 public class PIDToggleIntake extends PIDCommand {

   double m_maxSpeed, m_place, m_setPoint;
  boolean m_value;
  PIDController m_intakeAnalogPot;

   public PIDToggleIntake(double maxSpeed, double place, boolean value) {
    super(0.02, 0, 0);
    requires(Robot.m_intake);

     m_maxSpeed = maxSpeed;
    m_place = place;
    m_value = value;

   }

   // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    setSetpoint(m_setPoint);
    setInputRange(RobotConstants.MAXIMUM_IMPUT_RANGE, RobotConstants.MINIMUM_IMPUT_RANGE);
  }

   // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
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
  protected double returnPIDInput() {
    return Robot.m_intake.getAnalogPot().get();   
  }

   protected void usePIDOutput(double output){
    Robot.m_intake.getAnalogPotMotor().set(ControlMode.PercentOutput, output);

   }

 }
