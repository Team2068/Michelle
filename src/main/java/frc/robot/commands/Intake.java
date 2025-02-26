// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utility.IO;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Intake extends Command {

  Runnable intake;
  Runnable stop;
  BooleanSupplier holding;
  double angle;

  // public static int INTAKE_CORAL = -1;
  // public static int INTAKE_ALGAE = -2;
  // public static int SCORE_CORAL = 1;
  // public static int SCORE_ALGAE = 2;
  // public static int INTAKE_ALGAE_GROUND = -3; // THIS WILL MATTER ONLY IF WE DO GROUND PICKUP

  public Intake(IO io, boolean coral, boolean release) { // negative is release, 1 is coral, 2 is algae, 3 is algae in ground positio
    
    if (coral){
      intake = () -> io.claw.speedCoral( (release) ? .4 : -.4);
      stop = io.claw::stopCoral;
      holding = () -> (release) ? io.claw.hasCoral() :  !io.claw.hasCoral();
    } else { // Algae
      intake = () -> {
        io.claw.speedAlgae(1);
        // TODO: Set Claw to Reef intake Angle or Ground pickup if we
      };
      stop = io.claw::stopAlgae;
      holding = () -> (release) ? io.claw.hasAlgae() :  !io.claw.hasAlgae();
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.run();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stop.run();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return holding.getAsBoolean();
  }
}
