// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utility.IO;
import frc.robot.utility.LimelightHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LimelightAlign extends Command {

  IO io;
  int positions;
  PIDController pid = new PIDController(.1, 0, 0);
  String limelight = "limelight-main";

  public LimelightAlign(IO io, int positions) {
    this.io = io;
    this.positions = positions;
    pid.setTolerance(1);
    addRequirements(io.chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (positions) {
      case 2: // Right
    LimelightHelpers.SetFidcuial3DOffset(limelight, 0,0.2, 0);
        break;
      case 1: // Center
      LimelightHelpers.SetFidcuial3DOffset(limelight, 0, 0, 0);
        break;
      default:// Left
      LimelightHelpers.SetFidcuial3DOffset(limelight, 0, -0.2, 0);
        break;
    }
    LimelightHelpers.Flush();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    io.chassis.drive(new ChassisSpeeds(0, -pid.calculate(LimelightHelpers.getTX("limelight-main"), 0), 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    io.chassis.drive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return pid.atSetpoint() || LimelightHelpers.getTV("limelight-main");
    return pid.atSetpoint();
  }
}
