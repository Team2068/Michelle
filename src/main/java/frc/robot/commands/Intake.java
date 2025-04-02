// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utility.IO;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Intake extends Command {

  Runnable intake;
  Runnable stop;
  BooleanSupplier holding;
  double angle;
  boolean release;

  // public static int INTAKE_CORAL = -1;
  // public static int INTAKE_ALGAE = -2;
  // public static int SCORE_CORAL = 1;
  // public static int SCORE_ALGAE = 2;
  // public static int INTAKE_ALGAE_GROUND = -3; // THIS WILL MATTER ONLY IF WE DO GROUND PICKUP

  public Intake(IO io, boolean release, GenericHID controller) {
    holding = () -> (release) ? io.shooter.coral() :  !io.shooter.coral();
    
    intake = () -> {
      io.shooter.speed((release) ? 1 : .4);
    };

    stop = () -> {
      io.shooter.stopIntake();
      controller.setRumble(RumbleType.kBothRumble, 0.0);
    };

    this.release = release;
  }

  public Intake(IO io, boolean release){
    this(io, release, null);
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
