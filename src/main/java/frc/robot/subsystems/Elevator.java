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
import com.revrobotics.spark.config.SparkMaxConfig;

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

  SparkMaxConfig config = new SparkMaxConfig();
  
  boolean limitsActive = true;

  TrapezoidProfile profile = new TrapezoidProfile(new Constraints(100, 500));
  Timer time = new Timer();
  double target = 0;
  boolean stopped = true;

  PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

  final String[] level_layout = { "Rest", "L1", "L2", "L3", "L4", "Barge", "Low Algae", "High Algae" };

  // Target Heights
  public final double Rest = 0;
  public final double L0 = 7;
  public final double L1 = 27;
  public final double L2 = 47;
  public final double L3 = 67;
  public final double L4 = 127;
  public final double Barge = 0; // TODO: FIND BARGE
  public final double Low_Algae = 0; // TODO: FIND
  public final double High_Algae = 0; // TODO: FIND
  public final double MAX_HEIGHT = 0; // TODO: FIND

  public Elevator() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = 0.01;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kG = 0.0;

    config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.SoftwareLimitSwitch
        .withForwardSoftLimitEnable(false)
        .withForwardSoftLimitThreshold(3000.0)
        .withReverseSoftLimitEnable(false)
        .withReverseSoftLimitThreshold(0.0);

    lead.getConfigurator().apply(config);
    follow.setControl(new Follower(lead.getDeviceID(), true));
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

  public void disableLimits(){
    lead.getConfigurator().apply(new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(false).withReverseSoftLimitEnable(false));
    limitsActive = false;
  }

  public void enableLimits(){
    lead.getConfigurator().apply(new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true).withReverseSoftLimitEnable(true));
    limitsActive = true;
  }

  public void toggleLimits(){
    if (limitsActive)
      disableLimits();
    else
      enableLimits();
  }

  public void move(int level) {
    switch (level) {
      case 0:
        move(L0);
        break;
      case 1:
        move(L1);
        break;
      case 2:
        move(L2);
        break;
      case 3:
        move(L3);
        break;
      case 4:
        move(L4);
        break;
      case 5:
        move(Barge);
        break;
      case 6:
        move(Low_Algae);
        break;
      case 7:
        move(High_Algae);
        break;
      default:
        move(Rest); // LEVEL 1 // TODO: See if we need to change the height we go to
        break;
    }
    SmartDashboard.putString("Target Level", level_layout[level]);
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

    if (stopped)
      return;

    State out = profile.calculate(time.get(), new State(L2, Barge), new State(target, 0));
    lead.setControl(positionRequest.withPosition(out.position));

    SmartDashboard.putNumber("Elevator Motor Voltage", voltage().magnitude());
    // SmartDashboard.putNumber("Elevator Height", position());

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
