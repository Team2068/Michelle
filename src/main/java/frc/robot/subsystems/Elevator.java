// package frc.robot.subsystems;

// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;

// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.*;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Elevator extends SubsystemBase {
//   public SparkMax motor = new SparkMax(7, MotorType.kBrushless);;
//   public SparkMax follower = new SparkMax(8, MotorType.kBrushless);
//   SparkMaxConfig config = new SparkMaxConfig();

//   TrapezoidProfile profile = new TrapezoidProfile(new Constraints(100, 500));
//   Timer time = new Timer();
//   double target = 0;
//   boolean stopped = true;

//   // Target Heights
//   public final double Rest = 0;
//   public final double L2 = 0;
//   public final double L3 = 0;
//   public final double L4 = 0;
//   public final double Barge = 0;
//   public final double MAX_HEIGHT = 0;


//   public Elevator() {
//     config
//       .idleMode(SparkMaxConfig.IdleMode.kBrake)
//       .closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
//       .pid(0, 0, 0);
    
//     motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

//     config.follow(motor, true);

//     follower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

//     zero();
//   }

//   public void speed(double speed) {
//     motor.set(speed);
//   }
  
//   public void volts(double volts) {
//     motor.setVoltage(volts);
//   }

//   public void stop(){
//     stopped = true;
//     motor.stopMotor();
//   }

//   public void movePID(double height) {
//     motor.getClosedLoopController().setReference(height, ControlType.kPosition);
//   }

//   public void move(double height){ // TODO: Checkout how adding a feedforward affects the results
//     stopped = false;
//     target = Math.max( Math.min( height, 0 ), MAX_HEIGHT );
//     time.restart();
//   }

//   public InstantCommand move(int level){
//     return new InstantCommand(() -> {
//       switch (level) {
//         case 2: L2();
//           break;
//         case 3: L3();
//           break;
//         case 4: L4();
//           break;
//         case 5: Barge();
//         break;
//         default: rest(); // LEVEL 1 // TODO: See if we need to change the height we go to
//           break;
//       }
//     },this);
//   }

//   public double elevatorPos() {
//     return motor.getEncoder().getPosition();
//   }

//   public void zero(){
//     motor.getEncoder().setPosition(0);
//   }

//   public void Rest(){
//     move(Rest);
//   }

//   public void L2(){
//     move(L2);
//   }

//   public void L3(){
//     move(L3);
//   }

//   public void L4(){
//     move(L4);
//   }

//   // @Override
//   // public void periodic() {
//   //   if (stopped) return;

//   //   State out = profile.calculate(time.get(), new State(L2, Barge), new State(target, 0));
//   //   motor.getClosedLoopController().setReference(out.position, ControlType.kPosition);
    
//   //   // TODO: Maybe log Supplied Volts
//   //   SmartDashboard.putNumber("Elevator Height", motor.getEncoder().getPosition());
    
//   //   SmartDashboard.putNumber("Elevator Target Height", target);
//   //   SmartDashboard.putNumber("Elevator cTarget Height", out.position);

//   //   SmartDashboard.putNumber("Elevator Speed", motor.getEncoder().getVelocity());
//   //   SmartDashboard.putNumber("Elevator cTarget Velocity", out.velocity);
//   // }
// }
