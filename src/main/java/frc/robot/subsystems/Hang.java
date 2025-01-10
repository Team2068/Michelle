package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hang extends SubsystemBase {
  public TalonFX hang = new TalonFX(0, "rio");

  public Hang() {
    hang.setNeutralMode(NeutralModeValue.Brake);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = 0;
    config.Slot0.kP = 0;
    config.Slot0.kP = 0;
    config.CurrentLimits = new CurrentLimitsConfigs().withStatorCurrentLimitEnable(true).withStatorCurrentLimit(20);
    hang.getConfigurator().apply(config);
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

  public double getHangPos() {
    return hang.getPosition().getValueAsDouble();
  }

  public void setHangPos(double pos) {
    hang.setControl(new PositionVoltage(pos).withSlot(0));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hang Pos", getHangPos());
  }
}
