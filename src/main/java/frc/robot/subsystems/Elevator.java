package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
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

  LiDARDio lidar = new LiDARDio(4);

  boolean selectAbsolute = true; // NOTE: CAN SET TO FALSE IN CASE OF EMERGENCIES LIKE CORAL BLOCKING LIDAR

  TrapezoidProfile profile = new TrapezoidProfile(new Constraints(100, 500));
  public boolean stopped = true;
  Timer time = new Timer();
  double target = 0;

  PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

  final String[] levelLayout = { "Rest", "L1", "L2", "L3", "L4", "Barge", "Low Algae", "High Algae" };

  public final double[] Level = { 0, 25, 43.5, 76, 110 };
  public final double Rest = 0;
  public final double L1 = 25;
  public final double L2 = 43.5;
  public final double L3 = 76;
  public final double L4 = 110;

  public boolean softLimits = false;

  public Elevator() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = 0.3;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.1;
    config.Slot0.kG = 0.0;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.SoftwareLimitSwitch
        .withForwardSoftLimitEnable(softLimits)
        .withForwardSoftLimitThreshold(160.0)
        .withReverseSoftLimitEnable(softLimits)
        .withReverseSoftLimitThreshold(0.0);

    lead.getConfigurator().apply(config);
    follow.setControl(new Follower(lead.getDeviceID(), true));
  }

  public void toggleSoftLimits() {
    softLimits = !softLimits;
    lead.getConfigurator().apply(
        new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(softLimits).withReverseSoftLimitEnable(softLimits));
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
    follow.stopMotor();
  }

  public void move(double height) {
    stopped = false;
    target = height;
    time.restart();
  }

  public void move(int level) {
    move(Level[Math.min(0, Math.max(4, level))]);
    SmartDashboard.putString("Target Elevator Level", levelLayout[level]);
  }

  public InstantCommand moveCommand(int level) {
    return new InstantCommand(() -> this.move(level), this);
  }

  public void zero() {
    lead.setPosition(0);
  }

  public boolean atPosition() {
    return profile.isFinished(time.get());
  }

  public Voltage voltage() {
    return lead.getMotorVoltage().getValue();
  }

  public double position() {
    // return lead.getPosition().getValueAsDouble() * conversion;
    return lead.getPosition().getValueAsDouble(); 
  }

  public boolean absoluteConnected(){
    return lidar.connected();
  }

  public LinearVelocity velocity() {
    // return MetersPerSecond.of(lead.getVelocity().getValueAsDouble() *
    // conversion);
    return MetersPerSecond.of(lead.getVelocity().getValueAsDouble());
  }

  final double gearReduction = 1 / 17;
  final double conversion = Math.PI * gearReduction * (Units.inchesToMeters(2) / 60);

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
    SmartDashboard.putNumber("Elevator Height", position());
    SmartDashboard.putNumber("Elevator Height LiDAR", lidar.distance());
    SmartDashboard.putBoolean("Elevator LiDAR Connected", absoluteConnected());
    SmartDashboard.putBoolean("Elevator Soft Limits Active", softLimits);

    double cTime = time.get();

    if (stopped)
      return;

    if(absoluteConnected() && selectAbsolute) {
      lead.setPosition(lidar.distance());
    }

    State out = profile.calculate(cTime, new State(L2, lead.getVelocity().getValueAsDouble()), new State(target, 0));
    lead.setControl(positionRequest.withPosition(out.position).withEnableFOC(true));

    stopped = profile.isFinished(cTime);

    SmartDashboard.putNumber("Elevator Motor Voltage", voltage().magnitude());

    SmartDashboard.putNumber("Elevator Target Height", target);
    SmartDashboard.putNumber("Elevator cTarget Height", out.position);

    SmartDashboard.putNumber("Elevator Velocity", velocity().magnitude());
    SmartDashboard.putNumber("Elevator cTarget Velocity", out.velocity);
  }
}
