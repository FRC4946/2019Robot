/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.Limelight.SetLimelightLED;
import frc.robot.commands.drivetrain.AlignWithTarget;

public class TestAuto extends CommandGroup {
  /**
   * Add your docs here.
   */
  public TestAuto() {
    addSequential(new SetLimelightLED(true));
    addSequential(new AlignWithTarget());
  }
}
