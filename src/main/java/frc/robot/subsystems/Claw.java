package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {

  Servo stopper = new Servo(0);
  Servo slapper = new Servo(1);

  public boolean open = false;
  public boolean slapping = false;

  public void open(){
    stopper.setAngle(0); //TODO: Find open & closed Angles
    open = true;
  }

  public void close(){
    stopper.setAngle(0);
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

  public void toggleStopper(){
    stopper.setAngle((open) ? 0.0 : 0.0);
  }

  public void toggleSlapper(){
    slapper.setAngle((slapping) ? 0.0 : 0.0);
  }
  
  

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Coral Stopper Angle", stopper.getAngle());
    SmartDashboard.putBoolean("Coral Stopper Open", open);

    SmartDashboard.putNumber("Slapper Angle", stopper.getAngle());
    SmartDashboard.putBoolean("Slapping", open);
  }
}
