package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

public class Elevator extends SubsystemBase {
  TalonFX lead = new TalonFX(11);
  TalonFX follow = new TalonFX(12);

  SparkMaxConfig config = new SparkMaxConfig();

  TrapezoidProfile profile = new TrapezoidProfile(new Constraints(100, 500));
  Timer time = new Timer();
  double target = 0;
  boolean stopped = true;

  PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

  // Target Heights
  public final double Rest = 0;
  public final double L1 = 0;
  public final double L2 = 0;
  public final double L3 = 0;
  public final double L4 = 0;
  public final double Barge = 0;
  public final double MAX_HEIGHT = 0;

  public Elevator() {
    Slot0Configs config = new Slot0Configs();
    config.kP = 0.0;
    config.kI = 0.0;
    config.kD = 0.0;
    config.kG = 0.0;

    lead.getConfigurator().apply(config);
    // lead.getConfigurator().apply(new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true).withReverseSoftLimitEnable(true)
    // .withForwardSoftLimitThreshold(0).withReverseSoftLimitThreshold(0)); // TODO: Find the forward and reverse Soft limit switches
    follow.setControl(new Follower(lead.getDeviceID(), false)); // TODO: Check if we need to invert
  }

  public void speed(double speed) {
    lead.set(speed);
  }

  public void volts(double volts) {
    lead.setVoltage(volts);
  }

  public void stop() {
    stopped = true;
    lead.stopMotor();
  }

  public void move(double height) {
    stopped = false;
    // target = Math.max(Math.min(height, 0), MAX_HEIGHT);
    target = height;
    time.restart();
  }

  public InstantCommand move(int level) {
    return new InstantCommand(() -> {
      switch (level) {
        case 2:
          L2();
          break;
        case 3:
          L3();
          break;
        case 4:
          L4();
          break;
        case 5:
          Barge();
          break;
        default:
          rest(); // LEVEL 1 // TODO: See if we need to change the height we go to
          break;
      }
    }, this);
  }

  public void zero() {
    lead.setPosition(0);
  }

  public void rest() {
    move(Rest);
  }
  public void L1(){
    move(L1);
  }

  public void L2() {
    move(L2);
  }

  public void L3() {
    move(L3);
  }

  public void L4() {
    move(L4);
  }

  public void Barge() {
    move(Barge);
  }

  public boolean atPosition() {
    return profile.isFinished(time.get());
  }

  public Voltage voltage() {
    return lead.getMotorVoltage().getValue();
  }

  public double position() {
    return lead.getPosition().getValueAsDouble() * conversion;
  }

  public LinearVelocity velocity() {
    return MetersPerSecond.of(lead.getVelocity().getValueAsDouble() * conversion);
  }

  final double gearReduction = 1/17;
  final double conversion = Math.PI * gearReduction *(Units.inchesToMeters(2) / 60);

  public final SysIdRoutine routine = new SysIdRoutine(new Config(
      null,
      Volts.of(4),
      Seconds.of(5),
      (state) -> SignalLogger.writeString("state", state.toString())),
      new Mechanism(
          volts -> lead.setControl(new VoltageOut(volts.in(Volts))),
          log -> {
            log.motor("Elevator")
                .voltage(voltage())
                .linearPosition(Meters.of(position()))
                .linearVelocity(velocity());
          }, this));

  @Override
  public void periodic() {
    if (stopped)
      return;

    State out = profile.calculate(time.get(), new State(L2, Barge), new State(target, 0));
    lead.setControl(positionRequest.withPosition(out.position));

    SmartDashboard.putNumber("Elevator Motor Voltage", voltage().magnitude());
    SmartDashboard.putNumber("Elevator Height", position());

    SmartDashboard.putNumber("Elevator Target Height", target);
    SmartDashboard.putNumber("Elevator cTarget Height", out.position);

    SmartDashboard.putNumber("Elevator Velocity", velocity().magnitude());
    SmartDashboard.putNumber("Elevator cTarget Velocity", out.velocity);

    // DogLog.log("Elevator/Height", motor.getEncoder().getPosition());
    // DogLog.log("Elevator/Target Height", target);
    // DogLog.log("Elevator/cTarget Height", out.position);
    // DogLog.log("Elevator/Speed", motor.getEncoder().getVelocity());
    // DogLog.log("Elevator/cTarget Velocity", out.velocity);
  }
}
