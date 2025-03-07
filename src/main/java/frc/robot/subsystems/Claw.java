package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {

  Servo closer = new Servo(0);
  Servo slapper = new Servo(1);

  public boolean open = false;
  public boolean slapping = false;

  public void open(){
    closer.setAngle(0); //TODO: Find open & closed Angles
    open = true;
  }

  public void close(){
    closer.setAngle(0);
    open = false; 
  }

  public void SlapReef(){
    slapper.setAngle(0);
    slapping = true;
  }

  public void stopSlapper(){
    slapper.setAngle(0);
    slapping = false;
  }
  
  

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Coral Stopper Angle", closer.getAngle());
    SmartDashboard.putBoolean("Coral Stopper Open", open);

    SmartDashboard.putNumber("Slapper Angle", closer.getAngle());
    SmartDashboard.putBoolean("Slapping", open);
  }
}
