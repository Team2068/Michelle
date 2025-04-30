package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.Util;

public class Hang extends SubsystemBase {
  // public TalonFX hang = new TalonFX(16, "rio"); // We Don't actually know the motor yet
  SparkMax motor = new SparkMax(16, MotorType.kBrushed); // CIM
  public SparkMaxConfig config = new SparkMaxConfig();

  public boolean up = false;
  public static final double upPosition = (double) Util.get("Hang Up Position", 0);
  public static final double downPosition = (double) Util.get("Hang Down Position", 0);
  
  public static final double kP = (double) Util.get("Hang kP", 0.0);
  public static final double kI = (double) Util.get("Hang kI", 0.0);
  public static final double kD = (double) Util.get("Hang kD", 0.0);

  public Hang() {
    config.idleMode(SparkMaxConfig.IdleMode.kBrake);
    config.closedLoop.pid(kP, kI, kD, ClosedLoopSlot.kSlot0);

    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void PID(double k, double i, double p){
    config.closedLoop.pid(k, i, p);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void speed(double speed) {
    motor.set(speed);
  }

  public void volts(double volts) {
    motor.setVoltage(volts);
  }

  public void stop() {
    motor.stopMotor();
  }

  public void toggleHang(){
    up = !up;
    motor.getClosedLoopController().setReference(((up) ? upPosition : downPosition), ControlType.kPosition);
  }

  public double angle() {
    return motor.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hang Position", motor.getEncoder().getPosition());
  }
}
