package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
  public SparkMax roller = new SparkMax(9, MotorType.kBrushless);
  public SparkMax pivot = new SparkMax(11, MotorType.kBrushless);
  public DigitalInput beamBreak = new DigitalInput(0);
  SparkMaxConfig rollerConfig = new SparkMaxConfig();
  SparkMaxConfig pivotConfig = new SparkMaxConfig();
  public DutyCycleEncoder encoder = new DutyCycleEncoder(0);

  public static final double CLOSED_ANGLE = 0;
  public boolean closed;

  public AlgaeIntake() {
    rollerConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
    roller.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pivotConfig.idleMode(SparkMaxConfig.IdleMode.kCoast);
    pivot.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    closed = (angle() > CLOSED_ANGLE);
  }

  public void IntakeVolts(double volts){
    roller.setVoltage(volts);
  }

  public void intakeSpeed(double speed){
    roller.set(speed);
  }

  public void pivotSpeed(double speed){
    pivot.set(speed);
  }

  public void pivotVolts(double volts){
    pivot.setVoltage(volts);
  }

  public void stop(){
    roller.stopMotor();
  }

  public boolean grabbed(){
    return beamBreak.get();
  }

  public double angle(){
    return encoder.get() * 360;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake Full", grabbed());
    closed = (angle() < CLOSED_ANGLE);
    SmartDashboard.putNumber("Pivot Angle", angle());
    // DogLog.log("Intake/Full", grabbed());
  }
}
