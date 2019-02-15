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

public class ActuateArmAndSetGrabber extends CommandGroup {
/**
 * Autonomus code for the grabber to function without human imput
 */
  public ActuateArmAndSetGrabber(boolean setIn, double speed) {
    
    //note: .cancel() feature should be available??

    if(setIn) {
      addParallel(new SetArmToPos(RobotConstants.GRABBER_ARM_OUT, speed));
      addSequential(new SetGrabber(true, speed));
    } else {
      addSequential(new SetGrabber(false, speed));
    }
  }
}
