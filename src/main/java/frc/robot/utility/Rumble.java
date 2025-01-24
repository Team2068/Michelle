// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.*;

public class Rumble extends Command {

Timer time = new Timer();
double duration = 0;
boolean condition = false;
Runnable action;
GenericHID controller;
RumblePatterns pattern = new RumblePatterns();
int type;
 
  public Rumble(int type, double duration, GenericHID controller, Runnable action) {
    time.start();
    this.type = type;
    this.controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // pattern.timer.start();
    action.run();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pattern.Run(type, controller);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    controller.setRumble(RumbleType.kBothRumble, 0);
    action.run();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return time.hasElapsed(duration) || condition;
  }
}
