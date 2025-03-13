// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.utility.IO;
import frc.robot.utility.Util;

public class ScoreReef extends SequentialCommandGroup {

  public ScoreReef(IO io, int level, GenericHID controller) {
    addCommands(
      new Intake(io, true, level, controller), // TODO: Set to Reef Scoring Angle
      io.elevator.moveCommand(0)
    );
  }

  public ScoreReef(IO io, boolean score_right, int level) {
    addCommands(
      // TODO: Check if we have coral & if we don't have
      io.elevator.moveCommand(level),
      new AutoAlign((score_right) ? 2 : 0, io),
      Util.Do(() -> io.claw.angle(level), io.claw),
      new WaitUntilCommand(io.elevator::atPosition), // TODO: Wait until we're at the height
      new Intake(io, true, level), // TODO: Set to Reef Scoring Angle
      io.elevator.moveCommand(0)
    );
  }
}
