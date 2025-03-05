// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.utility.IO;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Intake extends Command {

  Runnable intake;
  Runnable stop;
  Runnable rumble;
  BooleanSupplier holding;
  double angle;

  // public static int INTAKE_CORAL = -1;
  // public static int INTAKE_ALGAE = -2;
  // public static int SCORE_CORAL = 1;
  // public static int SCORE_ALGAE = 2;
  // public static int INTAKE_ALGAE_GROUND = -3; // THIS WILL MATTER ONLY IF WE DO GROUND PICKUP

  public Intake(IO io, boolean coral, boolean release, GenericHID controller) {
    rumble = (controller != null) ? () -> controller.setRumble(RumbleType.kBothRumble, .25) : () -> {}; // TODO: Check if it's fine
    holding = () -> (coral) ? ((release) ? io.claw.hasCoral() :  !io.claw.hasCoral()) : ((release) ? io.claw.hasAlgae() :  !io.claw.hasAlgae());
    
    intake = () -> {
      io.claw.speed((release) ? 1 : .4);
      io.claw.angle((release) ? Claw.REEF_ANGLE : Claw.INTAKE_ANGLE); // TODO: See if we need to add an angle for scoring on L4 & Barge
    };

    stop = () -> {
      io.claw.stop();
      controller.setRumble(RumbleType.kBothRumble, 0.0);
    };
  }

  public Intake(IO io, boolean coral, boolean release){
    this(io, coral, release, null);
  }

  public Intake(IO io, boolean coral, boolean release, int level, GenericHID controller) { // negative is release, 1 is coral, 2 is algae, 3 is algae in ground positio
    this(io, coral, release,controller);
    io.elevator.move(level);
  }

  public Intake(IO io, boolean coral, boolean release, int level) { // negative is release, 1 is coral, 2 is algae, 3 is algae in ground positio
    this(io, coral, release);
    io.elevator.move(level);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.run();
    rumble.run();
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
