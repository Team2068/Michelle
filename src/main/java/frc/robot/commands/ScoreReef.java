// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.utility.IO;
import frc.robot.utility.Util;

public class ScoreReef extends SequentialCommandGroup {

  public ScoreReef(IO io, int reefPosition, int level) {
    addCommands(
      new LimelightAlign(io, reefPosition, false),
      io.elevator.moveCommand(level),
      new WaitUntilCommand(io.elevator::atPosition),
      Util.Do(() -> io.claw.angle(level), io.claw),
      new Intake(io, true), // TODO: Set to Reef Scoring Angle
      io.elevator.moveCommand(0)
    );
  }
}
