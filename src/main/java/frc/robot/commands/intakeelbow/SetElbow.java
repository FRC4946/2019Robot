/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intakeelbow;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotConstants;
import frc.robot.commands.elevator.MoveToHeight;

public class SetElbow extends CommandGroup {
  /**
   * Add your docs here.
   */
  public SetElbow(boolean setToUp, double speed) {

    if(setToUp) {
      addSequential(new MoveToHeight(RobotConstants.ELEVATOR_MINIMUM_HEIGHT + 5.0, speed)); //inches
      addSequential(new SetOuterIntakeElbow(true, speed));
      addSequential(new MoveToHeight(RobotConstants.ELEVATOR_MINIMUM_HEIGHT, speed));
    } else {
      addSequential(new MoveToHeight(RobotConstants.ELEVATOR_MINIMUM_HEIGHT + 5.0, speed)); //inches
      addSequential(new SetOuterIntakeElbow(false, speed));
      addSequential(new MoveToHeight(RobotConstants.ELEVATOR_MINIMUM_HEIGHT, speed));
    }
  }
}
