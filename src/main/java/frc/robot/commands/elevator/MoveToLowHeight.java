/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.RobotConstants;
import frc.robot.commands.grabber.SetGrabber;
import frc.robot.commands.grabberarm.SetArmToPos;
import frc.robot.commands.intakeelbow.SetIntakePos;

public class MoveToLowHeight extends CommandGroup {
  /**
   * Add your docs here.
   */
  public MoveToLowHeight() { 
    addSequential(new SetGrabber(true, 0.8));
    addSequential(new MoveToHeight(RobotConstants.ELEVATOR_RIGHT_ABOVE_ELBOW + 0.5, 0.6, false), 4.0);
    
    addSequential(new SetIntakePos(RobotConstants.INTAKE_POT_BALL_HEIGHT, 0.25));
    addSequential(new MoveToHeight(RobotConstants.ELEVATOR_AT_MIN, 0.6), 5.0);
  }
}