// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.Intake;
import frc.robot.commands.RotateChassis;
import frc.robot.commands.ScoreReef;
import frc.robot.swerve.Swerve;
import frc.robot.utility.AutomatedController;
import frc.robot.utility.IO;
import frc.robot.utility.Util;

public class RobotContainer {
  
  public IO io = new IO();
  public final AutomatedController main;
  public final AutomatedController backup;

  private final SendableChooser<Command> auto_selector;
  Command current_auto = new PrintCommand("");
  final SendableChooser<Integer> driver_selector = new SendableChooser<Integer>();

  public RobotContainer() {
    main = new AutomatedController(0, io);
    backup = new AutomatedController(1, io);

    auto_selector = AutoBuilder.buildAutoChooser();
    auto_selector.onChange((command) -> {current_auto = command;});

    SmartDashboard.putData("Autos", auto_selector);
    SmartDashboard.putData("Run Test Auto", Util.Do(current_auto::schedule));

    SmartDashboard.putData("Main-Controller Mode", main.selector);
    SmartDashboard.putData("Backup-Controller Mode", backup.selector);
    SmartDashboard.putData("Driver", driver_selector);
    io.chassis.setDefaultCommand(new DefaultDrive(io, main.controller));

    driver_selector.setDefaultOption("Shaan", 0);
    driver_selector.addOption("Norah", 1);
    driver_selector.addOption("Jason", 2);
    driver_selector.addOption("Uriel", 3);
    driver_selector.addOption("Debug", 4);
    driver_selector.onChange( (driver) -> Swerve.Constants.SwitchDriver(driver));

    // DogLog.setOptions(new DogLogOptions()
    // .withCaptureDs(true)
    // .withCaptureConsole(true)
    // .withLogExtras(true));

    // DogLog.setEnabled( false); // TODO: Turn back on when we're testing proper
    SignalLogger.setPath("/media/sda1/ctre-logs/");
  }


  public void configureAuton(){
    NamedCommands.registerCommand("Face Barge", new RotateChassis(io, 0));
    NamedCommands.registerCommand("Rotate IJ", new RotateChassis(io, -120));
    NamedCommands.registerCommand("Rotate LK", new RotateChassis(io, -60));
    NamedCommands.registerCommand("Rotate HG", new RotateChassis(io, 180));
    NamedCommands.registerCommand("Rotate CD", new RotateChassis(io, 60));
    NamedCommands.registerCommand("Rotate EF", new RotateChassis(io, 120));

    NamedCommands.registerCommand("Score L-L4", new ScoreReef(io, false, 4));
    NamedCommands.registerCommand("Score L-L3", new ScoreReef(io, false, 3));
    NamedCommands.registerCommand("Score L-L2", new ScoreReef(io, false, 2));
    NamedCommands.registerCommand("Score L-L1", new ScoreReef(io, false, 1));

    NamedCommands.registerCommand("Score R-L4", new ScoreReef(io, true, 4));
    NamedCommands.registerCommand("Score R-L3", new ScoreReef(io, true, 3));
    NamedCommands.registerCommand("Score R-L2", new ScoreReef(io, true, 2));
    NamedCommands.registerCommand("Score R-L1", new ScoreReef(io, true, 1));

    // TODO: SEE IF WE CAN GET AWAY WITH THESE LEVELS
    NamedCommands.registerCommand("Clear Low Algae", new Intake(io, false, 2)); // TODO: MAKE ACTUAL COMMAND
    NamedCommands.registerCommand("Clear High Algae", new Intake(io, false, 3)); // TODO: MAKE ACTUAL COMMAND
  }

  public Command getAutonomousCommand() {
    return auto_selector.getSelected();
  }
}
