// package frc.robot.subsystems;

// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import dev.doglog.DogLog;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Hang extends SubsystemBase {
//   public TalonFX hang = new TalonFX(0, "rio");

//   public static final double HANG_MAX_ANGLE = 0;

//   public Hang() {
//     hang.setNeutralMode(NeutralModeValue.Brake);
//   }

//   public void hangSpeed(double speed) {
//     hang.set(speed);
//   }

//   public void hangVoltage(double volts) {
//     hang.setVoltage(volts);
//   }

//   public void stopHang() {
//     hang.stopMotor();
//   }

//   // @Override
//   // public void periodic() {
//   //   SmartDashboard.putNumber("Hang Pos", hang.getPosition().getValueAsDouble());
//   // }
// }
