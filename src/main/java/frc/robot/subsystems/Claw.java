package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import dev.doglog.DogLog;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

public class Claw extends SubsystemBase {
  SparkMax algaeIntake = new SparkMax(13, MotorType.kBrushless);
  TalonFX coralIntake = new TalonFX(14);
  SparkMax pivot = new SparkMax(15, MotorType.kBrushless);
  
  DigitalInput algaeBreak = new DigitalInput(0);
  DigitalInput coralBreak = new DigitalInput(1);

  TrapezoidProfile profile = new TrapezoidProfile(new Constraints(100, 500));
  Timer time = new Timer();
  double target = 0;
  boolean stopped = true;

  public Claw() {
    SparkMaxConfig config = new SparkMaxConfig();

    config.idleMode(SparkMaxConfig.IdleMode.kBrake);
    algaeIntake.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.closedLoop.pidf(0.0, 0.0, 0.0, 0.0, ClosedLoopSlot.kSlot0);
    config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    pivot.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void voltsAlgae(double volts){
    algaeIntake.setVoltage(volts);
  }

  public void speedAlgae(double speed){
    algaeIntake.setVoltage(speed);
  }

  public void voltsCoral(double volts){
    coralIntake.setVoltage(volts);
  }

  public void speedCoral(double speed){
    coralIntake.setVoltage(speed);
  }

  public void scoreCoral(){
    coralIntake.set(.4);
  }

  public void stopCoral(){
    coralIntake.set(0);
  }

  public void scoreAlgae(){
    algaeIntake.set(1);
  }

  public void stopAlgae(){
    algaeIntake.set(0);
  }

  public void stop(){
    algaeIntake.stopMotor();
  }

  public boolean hasAlgae(){
    return algaeBreak.get();
  }

  public boolean hasCoral(){
    return coralBreak.get();
  }

  public Voltage voltage(){
    return Volts.of(pivot.getBusVoltage()); // TODO: Return Bus voltage
  }

  public double angle(){
    return pivot.getAbsoluteEncoder().getPosition();
  }

  public void pivotVolts(double volts){
    pivot.setVoltage(volts);
  }

  public void setAngle(double target_angle){
    target = target_angle;
    stopped = false;
    time.restart();
  }

  public final SysIdRoutine routine = new SysIdRoutine(new Config(
      null,
      Volts.of(4),
      Seconds.of(5),
      (state) -> SignalLogger.writeString("state", state.toString())
    ), new Mechanism(
      volts -> pivot.setVoltage(volts),
      log -> {
        log.motor("Claw Pivot")
        .voltage(voltage())
        .linearPosition(Meters.of(0)) // TODO: Replace rotations with meters for the KRakens
        .linearVelocity(MetersPerSecond.of(0)); // TODO: Replace rotations per second to meters per second for the KRakens
       }, this));

  @Override
  public void periodic() {
    DogLog.log("Claw/Algae Full", hasAlgae());
    DogLog.log("Claw/Coral Full", hasCoral());
    DogLog.log("Claw/Pivot Angle", angle());

    double cTime = time.get();
    if (stopped) return;

    State out = profile.calculate(cTime, new State(angle(), 0), new State(target, 0));
    pivot.getClosedLoopController().setReference(out.position, ControlType.kPosition);
    stopped = profile.isFinished(cTime); // TODO: check if we haven't introduced any weirdness with this
  }
}
