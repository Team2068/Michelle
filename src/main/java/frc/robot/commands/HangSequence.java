// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// // import frc.robot.subsystems.Hang;
// import frc.robot.utility.IO;

// public class HangSequence extends Command {
//   IO io;

//   public HangSequence(IO io) {
//     this.io = io;
//     addRequirements(io.hang);
//   }

//   @Override
//   public void initialize() {
//     io.hang.hangVoltage(12);
//   }

//   @Override
//   public void execute() {
//   }

//   @Override
//   public void end(boolean interrupted) {
//     io.hang.stopHang();
//   }

//   @Override
//   public boolean isFinished() {
//     // return io.chassis.getRoll() > Hang.HANG_MAX_ANGLE;
//     return false;
//   }
// }