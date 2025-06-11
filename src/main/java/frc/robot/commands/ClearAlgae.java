// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.utility.IO;
import frc.robot.utility.Util;

public class ClearAlgae extends SequentialCommandGroup {

  public ClearAlgae(IO io) {
    addCommands(
      io.elevator.moveCommand(0),
      new WaitUntilCommand(.2), // Wait to be at the bottom
      Util.Do(() -> {
        io.shooter.angle(0);
        io.shooter.speed(-1);
      }, io.shooter), // TODO: See if we need a seperate angle for algae clearing
      io.elevator.moveCommand(4), // TODO: See if we can get away with L3 height 
      new WaitUntilCommand(.5), // Wait to be at the Top
      Util.Do(() -> {
        io.shooter.speed(0);
      }, io.shooter),
      io.elevator.moveCommand(0) // TODO: See if we can get away with L3 height 
    );
  }
}
