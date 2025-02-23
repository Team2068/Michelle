// package frc.robot.subsystems;

// import com.ctre.phoenix6.hardware.TalonFX;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkMaxConfig;

// import dev.doglog.DogLog;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Claw extends SubsystemBase {
//   SparkMax algaeIntake = new SparkMax(9, MotorType.kBrushless);
//   TalonFX coralIntake = new TalonFX(9);
//   SparkMax pivot = new SparkMax(10, MotorType.kBrushless);
  
//   DigitalInput algaeBreak = new DigitalInput(0);
//   DigitalInput coralBreak = new DigitalInput(1);
//   SparkMaxConfig rollerConfig = new SparkMaxConfig();

//   TrapezoidProfile profile = new TrapezoidProfile(new Constraints(100, 500));
//   Timer time = new Timer();
//   double target = 0;
//   boolean stopped = true;

//   public Claw() {
//     rollerConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);

//     algaeIntake.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//   }

//   public void voltsAlgae(double volts){
//     algaeIntake.setVoltage(volts);
//   }

//   public void speedAlgae(double speed){
//     algaeIntake.setVoltage(speed);
//   }

//   public void voltsCoral(double volts){
//     coralIntake.setVoltage(volts);
//   }

//   public void speedCoral(double speed){
//     coralIntake.setVoltage(speed);
//   }

//   public void scoreCoral(){
//     coralIntake.set(.4);
//   }

//   public void stopCoral(){
//     coralIntake.set(0);
//   }

//   public void scoreAlgae(){
//     algaeIntake.set(1);
//   }

//   public void stopAlgae(){
//     algaeIntake.set(0);
//   }

//   public void stop(){
//     algaeIntake.stopMotor();
//   }

//   public boolean hasAlgae(){
//     return algaeBreak.get();
//   }

//   public boolean hasCoral(){
//     return coralBreak.get();
//   }

//   @Override
//   public void periodic() {
//     DogLog.log("Claw/Algae Full", hasAlgae());
//     DogLog.log("Claw/Coral Full", hasCoral());
//   }
// }
