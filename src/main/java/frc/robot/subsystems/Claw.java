package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  public SparkMax claw = new SparkMax(0, MotorType.kBrushless);
  SparkMaxConfig clawConfig = new SparkMaxConfig();

  public Claw() {
    clawConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);

    claw.configure(clawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void openClaw(double speed){
    claw.set(speed);
  }

  public void closeClaw(double speed){
    claw.set(speed * -1);
  }

  public void stpClaw(){
    claw.stopMotor();
  }

  @Override
  public void periodic() {
    
  }
}
