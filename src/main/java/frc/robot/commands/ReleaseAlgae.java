package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utility.IO;

public class ReleaseAlgae extends Command {
  IO io;

  double speed;

  public ReleaseAlgae(IO io, boolean spit_out) {
    this.io = io;

    speed = (spit_out) ? -12 : 12;
    addRequirements(io.intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    io.intake.volts(speed);
  }

  @Override
  public void end(boolean interrupted) {
    io.intake.stop();
  }

  @Override
  public boolean isFinished() {
    return !io.intake.grabbed();
  }
}
