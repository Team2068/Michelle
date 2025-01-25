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
  double x;
  double y;
  double robotyaw;
  double latency;
  
  /** Creates a new Limelight. */
  public Limelight() {
    final NetworkTable table = NetworkTableInstance.getDefault().getTable("Limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tl = table.getEntry("tl");

    x = tx.getDouble(0.0);
    y= ty.getDouble(0.0);
    area= ta.getDouble(0.0);
    latency = tl.getDouble(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);              //MIGHT NEED TO CHANGE THIS
    SmartDashboard.putNumber("LimelightArea", area); 
    SmartDashboard.putNumber("Limelight Latency", latency); 
  }
}
