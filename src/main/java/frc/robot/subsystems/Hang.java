package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hang extends SubsystemBase {
  // public TalonFX hang = new TalonFX(16, "rio"); // We Don't actually know the motor yet
  SparkMax motor = new SparkMax(16, MotorType.kBrushed); // CIM

  public boolean up = false;
  double upPosition = 0; // TODO: FIND UP POSITION
  double downPosition = 0;

  public Hang() {
    motor.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
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
