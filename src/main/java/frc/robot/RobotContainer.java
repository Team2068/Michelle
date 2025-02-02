// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DefaultDrive;
import frc.robot.utility.AutomatedController;
import frc.robot.utility.IO;
import frc.robot.utility.SwerveConstants;

public class RobotContainer {

  public IO io = new IO();
  public final AutomatedController main;
  public final AutomatedController backup;


  public RobotContainer() {
    new SwerveConstants();
    main = new AutomatedController(0, io);
    backup = new AutomatedController(1, io);
    SmartDashboard.putData("Main-Controller Mode", main.selector);
    SmartDashboard.putData("Backup-Controller Mode", main.selector);
    io.chassis.setDefaultCommand(new DefaultDrive(io, main.controller));
    DogLog.setOptions(new DogLogOptions().withCaptureDs(true));
    // SmartDashboard.putData("Autonomous", ); // TBD
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
