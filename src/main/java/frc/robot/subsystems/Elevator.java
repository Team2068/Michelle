package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  public SparkMax elevatorMotor = new SparkMax(0, MotorType.kBrushless);;
  public SparkMax elevatorFollower = new SparkMax(1, MotorType.kBrushless);
  SparkMaxConfig config = new SparkMaxConfig();

  public Elevator() {


    config
      .follow(elevatorMotor, true)
      .idleMode(SparkMaxConfig.IdleMode.kBrake);

    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0, 0, 0);

      elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      elevatorFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void moveElevator(double speed) {
    elevatorMotor.set(speed);
  }
  
  public void moveElevatorVolts(double speed) {
    elevatorMotor.setVoltage(speed);
  }

  public void stopElevator(){
    elevatorMotor.stopMotor();
  }

  public double elevatorPos() {
    return elevatorMotor.getEncoder().getPosition();
  }

  public void resetEncoders(){
    elevatorMotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {

  }
}
