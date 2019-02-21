/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ClimberSeries extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ClimberSeries() {
    addSequential(new SetClimberHeight(15), 3.0);
  }
}
