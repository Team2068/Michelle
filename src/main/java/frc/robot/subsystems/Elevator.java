package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import dev.doglog.DogLog;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.*;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

public class Elevator extends SubsystemBase {
  // public SparkMax motor = new SparkMax(7, MotorType.kBrushless);;
  // public SparkMax follower = new SparkMax(8, MotorType.kBrushless);

  TalonFX lead = new TalonFX(7);
  TalonFX follow = new TalonFX(8);
  
  SparkMaxConfig config = new SparkMaxConfig();

  TrapezoidProfile profile = new TrapezoidProfile(new Constraints(100, 500));
  Timer time = new Timer();
  double target = 0;
  boolean stopped = true;

  PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

  // Target Heights
  public final double Rest = 0;
  public final double L2 = 0;
  public final double L3 = 0;
  public final double L4 = 0;
  public final double Barge = 0;
  public final double MAX_HEIGHT = 0;


  public Elevator() {
    // config
    //   .idleMode(SparkMaxConfig.IdleMode.kBrake)
    //   .closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    //   .pid(0, 0, 0);
    
    // motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // config.follow(motor, true);

    // follower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    Slot0Configs config = new Slot0Configs();
    config.kP = 0.0;
    config.kI = 0.0;
    config.kD = 0.0;

    lead.getConfigurator().apply(config);
    follow.setControl(new Follower(lead.getDeviceID(), false)); // TODO: Check if we need to invert
  }

  public void speed(double speed) {
    lead.set(speed);
  }
  
  public void volts(double volts) {
    lead.setVoltage(volts);
  }

  public void stop(){
    stopped = true;
    lead.stopMotor();
  }

  public void move(double height){ // TODO: Checkout how adding a feedforward affects the results
    stopped = false;
    target = Math.max( Math.min( height, 0 ), MAX_HEIGHT ); // TODO: Should be able to remove later because we know what heights we set the bot to
    time.restart();
  }

  public double position() {
    // return motor.getEncoder().getPosition();
    return lead.getPosition().getValueAsDouble(); // TODO: Convert to metres or inches
  }

  public void zero(){
    // motor.getEncoder().setPosition(0);
    lead.setPosition(0);
  }

  public void rest(){
    move(Rest);
  }

  public void L2(){
    move(L2);
  }

  public void L3(){
    move(L3);
  }

  public void L4(){
    move(L4);
  }

  private Voltage leaderVoltage(){
    return lead.getMotorVoltage().getValue();
  }

  public final SysIdRoutine routine = new SysIdRoutine(new Config(
      null,
      Volts.of(4),
      Seconds.of(5),
      (state) -> SignalLogger.writeString("state", state.toString())
    ), new Mechanism(
      (volts) -> lead.setControl(new VoltageOut(volts.in(Volts))),
       null,
      this));

  @Override
  public void periodic() {
    if (stopped) return;

    State out = profile.calculate(time.get(), new State(L2, Barge), new State(target, 0));
    // motor.getClosedLoopController().setReference(out.position, ControlType.kPosition);
    lead.setControl(positionRequest.withPosition(out.position));
    
    SmartDashboard.putNumber("Elevator Voltage [Lead]", leaderVoltage().magnitude());
    SmartDashboard.putNumber("Elevator Height", position());
    
    SmartDashboard.putNumber("Elevator Target Height", target);
    SmartDashboard.putNumber("Elevator Profile Target Height", out.position);

    SmartDashboard.putNumber("Elevator Speed",lead.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Profile Target Velocity", out.velocity);

    // DogLog.log("Elevator/Voltage [Lead]", leaderVoltage().magnitude())
    // DogLog.log("Elevator/Height", position());
    // DogLog.log("Elevator/Target Height", target);
    // DogLog.log("Elevator/Profiled Target Height", out.position);
    // DogLog.log("Elevator/Profiled Speed", lead.getVelocity().getValueAsDouble());
    // DogLog.log("Elevator/Profiled Target Velocity", out.velocity);
  }
}
