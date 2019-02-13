/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

 package frc.robot.commands.slider;

 import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

 public class SetSlider extends Command {

   private double m_speed;

   /**
    * Runs the slider at the set speed
    * @param speed the speed to run the slider at from 1.0 to -1.0
    */
   public SetSlider(double speed) {
     requires(Robot.m_slider);
     m_speed = speed;
   }

   // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

   // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_slider.set(m_speed);
  }

   // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

   // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_slider.stop();
  }

   // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
