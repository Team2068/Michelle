package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.utility.IO;

public class ToggleClaw extends Command {
  IO io;
  boolean open = false;

  public ToggleClaw(IO io) {
    this.io = io;
    addRequirements(io.claw);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    open = !open;
    double clawPosition = open ? Claw.CLAW_CLOSED_POS : Claw.CLAW_OPEN_POS;
    io.claw.setClawPos(clawPosition);
  }

  @Override
  public void end(boolean interrupted) {
    io.claw.stopClaw();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
