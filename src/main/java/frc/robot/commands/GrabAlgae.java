// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.utility.IO;

// public class GrabAlgae extends Command {
//   IO io;

//   public GrabAlgae(IO io) {
//     this.io = io;
//     addRequirements(io.algaeIntake);
//   }

//   @Override
//   public void initialize() {}

//   @Override
//   public void execute() {
//     io.algaeIntake.volts(12);
//   }

//   @Override
//   public void end(boolean interrupted) {
//     io.algaeIntake.stop();
//   }

//   @Override
//   public boolean isFinished() {
//     return io.algaeIntake.grabbed();
//   }
// }
