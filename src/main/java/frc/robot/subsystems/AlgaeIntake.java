package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
  public SparkMax roller = new SparkMax(9, MotorType.kBrushless);
  // public DigitalInput beamBreak = new DigitalInput(0);
  SparkMaxConfig rollerConfig = new SparkMaxConfig();

  public AlgaeIntake() {
    rollerConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);

    roller.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void volts(double volts) {
    roller.setVoltage(volts);
  }

  public void speed(double speed) {
    roller.setVoltage(speed);
  }

  public void stop() {
    roller.stopMotor();
  }

  public boolean grabbed() {
    // return beamBreak.get();
    return true;
  }

  // @Override
  // public void periodic() {
  //   // SmartDashboard.putBoolean("Intake Full", grabbed());
  // }
}
