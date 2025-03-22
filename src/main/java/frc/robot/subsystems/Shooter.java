package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

public class Shooter extends SubsystemBase {
  SparkMax intake = new SparkMax(13, MotorType.kBrushless);
  SparkMax pivot = new SparkMax(14, MotorType.kBrushless);
  DutyCycleEncoder encoder = new DutyCycleEncoder(new DigitalInput(3)); // or 4
  DigitalInput beambreak = new DigitalInput(1);
  Servo hood = new Servo(0);


  double[] pivotAngle = {0,0,0,0,0};
  double[] hoodAngle = {0,0,0,0,0};

  TrapezoidProfile profile = new TrapezoidProfile(new Constraints(100, 500));
  Timer time = new Timer();
  double target = 0.0;
  boolean stopped = true;

  public boolean intaking = false;

  public Shooter() {
    SparkMaxConfig config = new SparkMaxConfig();

    config.idleMode(SparkMaxConfig.IdleMode.kBrake);
    intake.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
    pivotConfig.closedLoop.pidf(0.0, 0.0, 0.0, 0.0, ClosedLoopSlot.kSlot0);

    pivotConfig.softLimit.forwardSoftLimitEnabled(false);
    pivotConfig.softLimit.forwardSoftLimit(0); // TODO: Find the Forward soft limit
    pivotConfig.softLimit.reverseSoftLimitEnabled(false);
    pivotConfig.softLimit.reverseSoftLimit(0); // TODO: Find the Reverse soft limit

    pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setHood(double v){
    hood.setPosition(v);
  }

  public void volts(double volts) {
    pivot.setVoltage(volts);
  }

  public void speed(double speed) {
    intake.setVoltage(speed);
  }

  public void stop() {
    intake.stopMotor();
  }

  public boolean hasCoral() {
    return beambreak.get();
  }

  public Voltage voltage() {
    return Volts.of(pivot.getBusVoltage());
  }

  public double angle() {
    // return encoder.get(); 
    return pivot.getEncoder().getPosition();
    // return (encoder.get()  * 360.0 ); 

  }

  public void angle(double target_angle) {
    target = target_angle;
    stopped = false;
    time.restart();
  }

  public void angle(int level){
    angle(pivotAngle[level]);
    hood.setAngle(hoodAngle[level]);
  }

  public void pivotVolts(double volts) {
    pivot.setVoltage(volts);
  }

  public final SysIdRoutine pivotRoutine = new SysIdRoutine(new Config(
      null,
      Volts.of(4),
      Seconds.of(5),
      null),
      new Mechanism(
          volts -> pivot.setVoltage(volts),
          log -> {
            log.motor("Claw Pivot")
                .voltage(voltage())
                .angularPosition(Degree.of(angle()))
                .angularVelocity(DegreesPerSecond.of(pivot.getAbsoluteEncoder().getVelocity()));
          }, this));

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Coral", hasCoral());
    SmartDashboard.putNumber("Claw Pivot", angle());
    SmartDashboard.putNumber("Claw Absolute Pivot", encoder.get() * 360);
    SmartDashboard.putNumber("Hood Angle", hood.get() * 180);

    double cTime = time.get();
    if (stopped)
      return;

    State out = profile.calculate(cTime, new State(angle(), 0), new State(target, 0));
    pivot.getClosedLoopController().setReference(out.position, ControlType.kPosition);
    stopped = profile.isFinished(cTime);
  }
}
