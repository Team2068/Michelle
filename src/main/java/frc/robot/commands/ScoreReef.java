// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.utility.IO;

public class ScoreReef extends SequentialCommandGroup {

  public ScoreReef(IO io, boolean score_right, int Level) {
    addCommands(
      // TODO: Check if we have coral & if we don't have
      io.elevator.move(Level),
      new AutoAlign((score_right) ? 2 : 1, io),
      new WaitUntilCommand(io.elevator::atPosition), // TODO: Wait until we're at the height
      new Intake(io, true, true), // TODO: Set to Reef Scoring Angle
      io.elevator.move(0)
    );
  }
}
