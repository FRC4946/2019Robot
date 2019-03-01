/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.grabberarm;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotConstants;
import frc.robot.commands.grabber.SetGrabber;

public class ReleaseHatch extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ReleaseHatch() {
    addSequential(new SetArmToPos(RobotConstants.GRABBER_ARM_OUT, 0.8), 1.5);
    addSequential(new SetGrabber(true, 0.8), 2.0);
  }
}
