package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utility.IO;

public class PivotIntake extends Command {
  IO io;
  boolean origin;

  public PivotIntake(IO io) {
    this.io = io;
    addRequirements(io.intake);
  }

  @Override
  public void initialize() {
    origin = (io.intake.angle() > io.intake.closedAngle);
    io.intake.pivotVolts(6 * (origin ? 1 : -1));
  }

  @Override
  public void execute() {
    io.intake.closed = (io.intake.angle() > io.intake.closedAngle);
  }

  @Override
  public void end(boolean interrupted) {
    io.intake.stopPivot();
  }

  @Override
  public boolean isFinished() {
    return io.intake.closed != origin;
  }
}
