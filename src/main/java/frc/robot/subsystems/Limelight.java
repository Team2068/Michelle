// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  double area;
  public double x;
  double y;
  double robotyaw;
  double latency;
  int tagNumber; 

  int[] tagLocation;
  /** Creates a new Limelight. */
  public Limelight() {
    final NetworkTable table = NetworkTableInstance.getDefault().getTable("Limelight");
    x = table.getEntry("tx").getDouble(0.0);
    y = table.getEntry("ty").getDouble(0.0);
    area = table.getEntry("ta").getDouble(0.0);
    latency =  table.getEntry("tl").getDouble(0.0);
    tagNumber = (int) table.getEntry("tid").getInteger(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);              //MIGHT NEED TO CHANGE THIS
    SmartDashboard.putNumber("LimelightArea", area); 
    SmartDashboard.putNumber("Limelight Latency", latency); 
  }

  public int[] TagLocationProcessing() {
    /*{red/blue, zone}
      {  1/2,     1-3}
    zone 1 = coral
    zone 2 = coral station
    zone 3 =  processor

    ** if returns 0 and 0 there is no tag in view  **
    */
    // return red or blue
    if (tagNumber > 0){
      if (tagNumber <= 11){
        tagLocation[0] = 1;
      }
      else {
        tagLocation[0] = 2;
      }
    
      // return zone
      if (tagNumber >=6 && tagNumber <= 11 || tagNumber >= 17 && tagNumber <= 22){
        tagLocation[1] = 1;
      }
      else if (tagNumber >=1 && tagNumber <= 2 || tagNumber >= 12 && tagNumber <= 13) {
        tagLocation[1] = 2;
      }
      else {
       tagLocation[1] = 3;
      }
    }
    else {
      tagLocation[0] = 0;
      tagLocation[1] = 0;
    }
    return tagLocation;
  }
}
