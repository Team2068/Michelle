package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utility.IO;

public class RotateChassis extends Command {
  IO io;
  private final PIDController rotationPID = new PIDController(0.01, 0.0, 0.001);

  private double targetAngle;
  double yaw;
  double error;

  public RotateChassis(IO io, double targetAngle) {
    this.io = io;
    addRequirements(io.chassis);
    this.targetAngle = targetAngle;
    rotationPID.setTolerance(1);
    rotationPID.enableContinuousInput(0, 360);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    yaw = io.chassis.getYaw();
    io.chassis.drive(new ChassisSpeeds(0, 0, rotationPID.calculate(yaw, targetAngle)));
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return Math.abs(rotationPID.getError()) < 1;
  }
}
