// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utility.IO;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class aimbot extends Command {

  IO io;
  PIDController pid = new PIDController(0.01, 0.00, 0.00);
  double tolerance = 5;

  public aimbot(IO io) {
    addRequirements(io.limelight, io.chassis);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    io.chassis.drive(new ChassisSpeeds(0, pid.calculate(io.limelight.x), 0));
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return Math.abs(pid.getError()) < tolerance;
    }
}
