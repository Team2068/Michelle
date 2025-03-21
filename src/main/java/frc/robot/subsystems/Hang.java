package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hang extends SubsystemBase {
  public SparkMax hang = new SparkMax(16, MotorType.kBrushed);
  TrapezoidProfile profile = new TrapezoidProfile(new Constraints(100, 500));
  Timer time = new Timer();
  double target = 0.0;

  public Hang() {
    SparkMaxConfig config = new SparkMaxConfig();

    config.idleMode(SparkMaxConfig.IdleMode.kBrake);
    config.closedLoop.pid(0.3, 0.0, 0.0, ClosedLoopSlot.kSlot0);
    hang.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void hangSpeed(double speed) {
    hang.set(speed);
  }

  public void hangVoltage(double volts) {
    hang.setVoltage(volts);
  }

  public void stopHang() {
    hang.stopMotor();
  }

  public double angle() {
    return hang.getAbsoluteEncoder().getPosition();
  }

  @Override
  public void periodic() {
    double cTime = time.get();
    State out = profile.calculate(cTime, new State(angle(), 0), new State(target, 0));
    hang.getClosedLoopController().setReference(out.position, ControlType.kPosition);
    SmartDashboard.putNumber("Hang Pos", angle());
  }
}
