// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.utility.IO;
import frc.robot.utility.Util;

public class ClearAlgae extends SequentialCommandGroup {
  /** Creates a new ClearAlgae. */
  public ClearAlgae(IO io, boolean align) {
    addCommands(
      new LimelightAlign(io, 0, false),
      Util.Do(io.claw::SlapReef, io.claw),
      new WaitCommand(.5),
      Util.Do(io.claw::stopSlapper, io.claw)
    );
  }

  public ClearAlgae(IO io) {
    addCommands(
      Util.Do(io.claw::SlapReef, io.claw),
      new WaitCommand(.5),
      Util.Do(io.claw::stopSlapper, io.claw)
    );
  }
}
