// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.utility.IO;
import frc.robot.utility.Util;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreReef extends SequentialCommandGroup {
  /** Creates a new ScoreReef. */
  public ScoreReef(IO io, int level) {
    addCommands(
      io.elevator.moveCommand(level),
      new WaitUntilCommand(io.elevator::atPosition),
      Util.Do(io.claw::open, io.claw),
      new WaitCommand(.5),
      Util.Do(io.claw::close, io.claw),
      io.elevator.moveCommand(0)
    );
  }

  public ScoreReef(IO io, boolean right, int level) {
    addCommands(
      io.elevator.moveCommand(level),
      new LimelightAlign(io, (right) ? 2 : 0 , true),
      // new WaitUntilCommand(io.elevator::atPosition),
      Util.Do(io.claw::open, io.claw),
      new WaitCommand(.5),
      Util.Do(io.claw::close, io.claw),
      io.elevator.moveCommand(0)
    );
  }
}
