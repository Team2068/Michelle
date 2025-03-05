// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Consumer;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.utility.IO;

// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAlgae extends ParallelCommandGroup {

  public ScoreAlgae(IO io, boolean barge, GenericHID controller) {
    addCommands( 
      //TODO: Check if we have Algae
      io.elevator.moveCommand((barge) ? 5 : 0),
      new AutoAlign(1, io),
      new WaitUntilCommand(io.elevator::atPosition),
      new Intake(io, false, true, controller),
      io.elevator.moveCommand(0)
    );
  }

  public ScoreAlgae(IO io, boolean barge) {
    addCommands( 
      //TODO: Check if we have Algae
      io.elevator.moveCommand((barge) ? 5 : 0),
      new AutoAlign(1, io),
      new WaitUntilCommand(io.elevator::atPosition),
      new Intake(io, false, true),
      io.elevator.moveCommand(0)
    );
  }
}
