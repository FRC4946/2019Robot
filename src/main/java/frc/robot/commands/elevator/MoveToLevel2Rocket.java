/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.commands.intakeelbow.SetIntakePos;

public class MoveToLevel2Rocket extends CommandGroup {

  private boolean startedLow;

  public MoveToLevel2Rocket() {

    startedLow = false;
    addSequential(new MoveToHeight(RobotConstants.ELEVATOR_LEVEL_2_ROCKET + 0.3, 0.4), 4.0); 

  }
}
