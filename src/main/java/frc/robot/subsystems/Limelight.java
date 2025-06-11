// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.IO;


public class Limelight extends SubsystemBase {
  double area;
  public double x;
  double y;
  double robotyaw;
  double latency;
  public int tag; 

  int[] tagLocation;

  final NetworkTable table = NetworkTableInstance.getDefault().getTable("Limelight");
  IO io;
  public Limelight() {
  }

  @Override
  public void periodic() {
    x = table.getEntry("tx").getDouble(0.0);
    y = table.getEntry("ty").getDouble(0.0);
    area = table.getEntry("ta").getDouble(0.0);
    latency =  table.getEntry("tl").getDouble(0.0);
    tag = (int) table.getEntry("tid").getInteger(0);
    
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);              //MIGHT NEED TO CHANGE THIS
    SmartDashboard.putNumber("LimelightArea", area); 
    SmartDashboard.putNumber("Limelight Latency", latency); 
  }

  // outputs a pid value to fix the rotation of the robot towards the intended tag
  // public void fixRotationAndLocation(PIDController pid,double poleX, double poleY) {
  //   double currentRotation = io.chassis.getYaw();
  //   double currentPositionX = io.chassis.pose().getX();
  //   double currentPositionY = io.chassis.pose().getY();
  //   double tagRotation = TagPose[io.limelight.tag][CENTRE][ROTATION];
    
  //   // sets the chassis moving towards the target pole
  //   io.chassis.drive(new ChassisSpeeds(pid.calculate(currentPositionX, poleX),pid.calculate(currentPositionY, poleY),  pid.calculate(currentRotation, tagRotation)));
  // }
  
  public boolean zoned(){
    return ((tag > 11) ? Alliance.Blue : Alliance.Red) == DriverStation.getAlliance().get();
  }

  public boolean reefZone(){
    return (tag >=6 && tag <= 11 || tag >= 17 && tag <= 22);
  }
}
