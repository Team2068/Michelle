// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utility.IO;
import frc.robot.utility.LimelightHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class aimbot extends Command {

  IO io;
  PIDController pid = new PIDController(0.1, 0.00, 0.00);

  public aimbot(IO io) {
    this.io = io;
    pid.setTolerance(10);
    addRequirements(io.limelight, io.chassis);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double output = -pid.calculate(LimelightHelpers.getTX("limelight-main"), 0);
    SmartDashboard.putNumber("PID Output", output);
    io.chassis.drive(new ChassisSpeeds(0, output, 0));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return pid.atSetpoint() || !LimelightHelpers.getTV("limelight-main");
    }
}
