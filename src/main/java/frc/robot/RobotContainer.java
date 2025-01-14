// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utility.IO;
import frc.robot.utility.Util;

public class RobotContainer {

  public IO io = new IO();

  private final SendableChooser<Command> auto_selector;
  private Command current_auto;

  public RobotContainer() {
    auto_selector = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autos",auto_selector);
    SmartDashboard.putData("Run Test Auto", Util.Do(() -> {current_auto.schedule();}));
    auto_selector.onChange((command) -> {current_auto = command;});
  }

  public Command getAutonomousCommand() {
    return auto_selector.getSelected();
  }
}
