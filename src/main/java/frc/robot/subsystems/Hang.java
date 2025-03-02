package frc.robot.subsystems;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SoftLimitConfig;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hang extends SubsystemBase {
  public TalonFX hang = new TalonFX(16, "rio"); // We Don't actually know the motor yet


  public Hang() {
    // TODO: Configure a soft limit switch
    hang.setNeutralMode(NeutralModeValue.Brake);
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

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hang Position", hang.getPosition().getValueAsDouble());
  }
}
