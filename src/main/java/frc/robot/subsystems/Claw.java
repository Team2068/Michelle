package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  public SparkMax claw = new SparkMax(0, MotorType.kBrushless);
  SparkMaxConfig clawConfig = new SparkMaxConfig();

  public final static double CLAW_OPEN_POS = 0;
  public final static double CLAW_CLOSED_POS = 0;

  public Claw() {
    clawConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
    clawConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0, 0, 0);

    claw.configure(clawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void clawVoltage(double volts){
    claw.setVoltage(volts);
  }

  public void clawSpeed(double speed){
    claw.setVoltage(speed);
  }

  public void stopClaw(){
    claw.stopMotor();
  }

  public void setClawPos(double position){
    claw.getClosedLoopController().setReference(position, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    
  }
}
