// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utility.IO;
import frc.robot.utility.Util;

public class SimpleAlign extends Command {
  IO io;
  boolean right;
  Timer time = new Timer();

  public SimpleAlign(IO io, boolean right) {
    this.io = io;
    this.right = right;
    addRequirements(io.chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.restart();
    io.chassis.drive(new ChassisSpeeds(0, ((right) ? 1 : -1) * (double) Util.get("Align Speed", 2.5), 0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    io.chassis.drive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return time.hasElapsed((double) Util.get("Align Time", .2));
  }
}
