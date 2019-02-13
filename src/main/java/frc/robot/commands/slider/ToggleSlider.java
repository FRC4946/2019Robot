/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.slider;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class ToggleSlider extends Command {
  /**
   * Toggles the slider from its furthest inwards position to its furthest outwards position
   */
  public ToggleSlider() {
    requires(Robot.m_slider);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_slider.set(Robot.m_slider.getSliderOut() ? -0.5 : 0.5);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return this.timeSinceInitialized() >= RobotConstants.SLIDER_ACTUATION_TIME;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_slider.stop();
    Robot.m_slider.setSliderOut(!Robot.m_slider.getSliderOut());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
