package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.utility.Util;

public class Shooter extends SubsystemBase {
  
  SparkMax intake = new SparkMax(13, MotorType.kBrushless);
  SparkMax pivot = new SparkMax(14, MotorType.kBrushless);
  public SparkMaxConfig config = new SparkMaxConfig();
  public SparkMaxConfig pivotConfig = new SparkMaxConfig();

  // DutyCycleEncoder encoder = new DutyCycleEncoder(new DigitalInput(3)); // or 4
  DoubleSupplier[] position;
  
  DigitalInput beam = new DigitalInput(1);
  int currentCoralSensor = 0;
  BooleanSupplier[] coral;
  String[] coralSensingDisplay = {"Beam Break", "RPM"};

  Servo hood = new Servo(0);

  boolean softLimits = false;

  boolean pivotRedundancy = false;

  public static final double[] pivotAngle = (double[]) Util.get("Pivot Angle", new double[] {0,0,0,0,0,0}); // last one FOR BARGE
  public static final double[] hoodAngle = (double[]) Util.get("Hood Angle", new double[] {0,0,0,0,0,0}); // last one FOR BARGE

  TrapezoidProfile profile = new TrapezoidProfile(new Constraints(100, 500));
  Timer time = new Timer();
  double target = 0.0;
  boolean stopped = true;

  public boolean intaking = false;

  public Shooter() {
    config.idleMode(SparkMaxConfig.IdleMode.kBrake);
    intake.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pivotConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
    pivotConfig.closedLoop.pid(0.0, 0.0, 0.0, ClosedLoopSlot.kSlot0);

    pivotConfig.softLimit.forwardSoftLimitEnabled(false);
    pivotConfig.softLimit.forwardSoftLimit(0); // TODO: Find the Forward soft limit
    pivotConfig.softLimit.reverseSoftLimitEnabled(false);
    pivotConfig.softLimit.reverseSoftLimit(0); // TODO: Find the Reverse soft limit

    pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    position = new DoubleSupplier[]{pivot.getEncoder()::getPosition, pivot.getAbsoluteEncoder()::getPosition};
    coral = new BooleanSupplier[]{beam::get}; //TODO: Have the second option be RPM sensing
  }

    public void toggleSoftLimits() {
    softLimits = !softLimits;
    pivotConfig.softLimit.forwardSoftLimitEnabled(softLimits);
    pivotConfig.softLimit.reverseSoftLimitEnabled(softLimits);
    pivot.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void pivotPID(double p, double i, double d){
    pivotConfig.closedLoop.pid(p, i, d, ClosedLoopSlot.kSlot0);
    pivot.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void hood(double v){
    hood.setPosition(v);
  }

  public void hoodSpeed(double v){
    hood.setSpeed(v);
  }

  public void volts(double v) {
    intake.setVoltage(v);
  }

  public void speed(double s) {
    intake.set(s);
  }

  public void pivotVolts(double v) {
    pivot.setVoltage(v);
  }

  public void pivotSpeed(double s) {
    pivot.set(s);
  }

  public void stopIntake() {
    intake.stopMotor();
  }

  public void stopPivot(){
    pivot.stopMotor();
  }

  public void stop(){
    intake.stopMotor();
    pivot.stopMotor();
  }

  public boolean coral() {
    return coral[currentCoralSensor].getAsBoolean();
  }

  // public boolean algae() {
  //   return beam.get();
  // }

  public Voltage voltage() {
    return Volts.of(pivot.getBusVoltage());
  }

  public double angle() {
    return position[absoluteConnected() ? 1 : 0].getAsDouble();
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

  public InstantCommand angleCommand(int level){
    return new InstantCommand(() -> {
      angle(pivotAngle[level]);
      hood.setAngle(hoodAngle[level]);
    }, this);
  }

  public void togglePivotRedundancy(){
    pivotRedundancy = !pivotRedundancy;
  }

  public boolean absoluteConnected(){
    return (pivot.getAbsoluteEncoder().getPosition() == -2000); // TODO: 2000 is just a dummy value, replace with actual value present when the cable is not connected
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
    SmartDashboard.putBoolean("Coral", coral());
    SmartDashboard.putString("Current Coral Sensing", coralSensingDisplay[currentCoralSensor]);
    
    SmartDashboard.putNumber("Pivot Angle", angle());
    SmartDashboard.putNumber("Hood Angle", hood.get() * 180);

    SmartDashboard.putNumber("Pivot Relative Angle", pivot.getEncoder().getPosition());
    SmartDashboard.putNumber("Pivot Absolute Angle", pivot.getAbsoluteEncoder().getPosition());

    SmartDashboard.putBoolean("Claw Absolute Encoder Connected", absoluteConnected());

    SmartDashboard.putBoolean("Shooter Pivot Soft Limits", softLimits);
    // TODO: Find out how to indicate if the Hood Servo is connected or not

    double cTime = time.get();
    if (stopped)
      return;

    State out = profile.calculate(cTime, new State(angle(), 0), new State(target, 0));
    pivot.getClosedLoopController().setReference(out.position, ControlType.kPosition);
    stopped = profile.isFinished(cTime);
  }
}
