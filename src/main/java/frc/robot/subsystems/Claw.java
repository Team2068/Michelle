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
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

public class Claw extends SubsystemBase {
  SparkMax intake = new SparkMax(13, MotorType.kBrushless);
  SparkFlex pivot = new SparkFlex(14, MotorType.kBrushless);

  public static final double INTAKE_ANGLE = 0;
  public static final double REEF_ANGLE = 0;

  DigitalInput coralBreak = new DigitalInput(0);

  TrapezoidProfile profile = new TrapezoidProfile(new Constraints(100, 500));
  Timer time = new Timer();
  double target = 0.0;
  boolean stopped = true;

  public boolean intaking = false;

  public Claw() {
    SparkMaxConfig config = new SparkMaxConfig();

    config.idleMode(SparkMaxConfig.IdleMode.kBrake);
    intake.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig pivotConfig = new SparkFlexConfig();
    pivotConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
    pivotConfig.closedLoop.pidf(0.0, 0.0, 0.0, 0.0, ClosedLoopSlot.kSlot0);
    pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    pivotConfig.softLimit.forwardSoftLimitEnabled(false);
    pivotConfig.softLimit.forwardSoftLimit(0); // TODO: Find the Forward soft limit
    pivotConfig.softLimit.reverseSoftLimitEnabled(false);
    pivotConfig.softLimit.reverseSoftLimit(0); // TODO: Find the Reverse soft limit

    pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void volts(double volts) {
    intake.setVoltage(volts);
  }

  public void speed(double speed) {
    intake.setVoltage(speed);
  }

  public void stop() {
    intake.stopMotor();
  }

  public boolean hasCoral() {
    return coralBreak.get();
  }

  public Voltage voltage() {
    return Volts.of(pivot.getBusVoltage());
  }

  public double angle() {
    return pivot.getAbsoluteEncoder().getPosition();
  }

  public void angle(double target_angle) {
    target = target_angle;
    stopped = false;
    time.restart();
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
    // DogLog.log("Claw/Algae Full", hasAlgae());
    // DogLog.log("Claw/Coral Full", hasCoral());
    // DogLog.log("Claw/Pivot Angle", angle());

    SmartDashboard.putBoolean("Coral", hasCoral());
    SmartDashboard.putNumber("Claw Pivot", angle());

    double cTime = time.get();
    if (stopped)
      return;

    State out = profile.calculate(cTime, new State(angle(), 0), new State(target, 0));
    pivot.getClosedLoopController().setReference(out.position, ControlType.kPosition);
    stopped = profile.isFinished(cTime); // TODO: check if we haven't introduced any weirdness with this
  }
}
