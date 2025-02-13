// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  double area;
  public double x;
  double y;
  double robotyaw;
  double latency;
  int tag; 

  public static final int LEFT = 0;
  public static final int CENTRE = 1;
  public static final int RIGHT = 2;

  public static final int X = 0;
  public static final int Y = 1;
  public static final int ROTATION = 2;
  public static final double[][][] TagPose = {
    { // TAG 1     **All locations in meters**All rotations in radians**
      // LEFT POLE      // CENTRE TAG   // RIGHT POLE
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
    },
    { // TAG 2
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
    }
  };

  // EXAMPLE: TagPOse[1][RIGHT][X]

  int[] tagLocation;

  final NetworkTable table = NetworkTableInstance.getDefault().getTable("Limelight");

  public Limelight() {}

  @Override
  public void periodic() {
    // x = table.getEntry("tx").getDouble(0.0);
    y = table.getEntry("ty").getDouble(0.0);
    area = table.getEntry("ta").getDouble(0.0);
    latency =  table.getEntry("tl").getDouble(0.0);
    tag = (int) table.getEntry("tid").getInteger(0);
    
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);              //MIGHT NEED TO CHANGE THIS
    SmartDashboard.putNumber("LimelightArea", area); 
    SmartDashboard.putNumber("Limelight Latency", latency); 
  }

  public boolean zoned(){
    return ((tag > 11) ? Alliance.Blue : Alliance.Red) == DriverStation.getAlliance().get();
  }

  public boolean reefZone(){
    return (tag >=6 && tag <= 11 || tag >= 17 && tag <= 22);
  }
}
