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
      {0.0, 0.0, 0.0}, {16.6972, 0.65532, 0.01745329}, {0.0, 0.0, 0.0}
    },
    { // TAG 2
    //  x    y  rotation
      {0.0, 0.0, 0.0}, {16.6972, 7.39648,   4.08406986}, {0.0, 0.0, 0.0}
    },
    { // TAG 3
      {0.0, 0.0, 0.0}, {11.56081, 8.05561,  4.7123883 }, {0.0, 0.0, 0.0}
    },
    { // TAG 4
      {0.0, 0.0, 0.0}, {9.27608,  6.137656, 0.0       }, {0.0, 0.0, 0.0}
    },
    { // TAG 5
      {0.0, 0.0, 0.0}, {9.27608,  1.914906, 0.0       }, {0.0, 0.0, 0.0}
    },
    { // TAG 6
      {13.5914892, 4.620133, 0.0}, {13.47445, 3.306318, 5.235987  }, {13.3068568, 4.784471, 0.0}
    },
    { // TAG 7
      {13.839698, 3.861562, 0.0}, {13.8905,  4.0259,   0.0       }, {13.839698, 4.190238, 0.0}
    },
    { // TAG 8
      {13.3068568, 3.2674306, 0.0}, {13.47445, 4.745482, 1.0471974 }, {13.5914892, 3.4317686, 0.0}
    },
    { // TAG 9
      {12.5253496, 3.4317686, 0.0}, {12.64336, 4.745482, 2.0943948 }, {12.8104392, 3.2674306, 0.0}
    },
    { // TAG 10
      {12.277598, 4.190238, 0.0}, {12.22731, 4.0259,   3.1415922 }, {12.277598, 3.861562, 0.0}
    },
    { // TAG 11
      {12.8104392, 4.784471, 0.0}, {12.64336, 3.306318, 4.1887896 }, {12.5258068, 4.620133, 0.0}
    },
    { // TAG 12
      {0.0, 0.0, 0.0}, {0.851154, 0.65532,  0.94247766}, {0.0, 0.0, 0.0}
    },
    { // TAG 13
      {0.0, 0.0, 0.0}, {0.851154, 7.39648,  5.34070674}, {0.0, 0.0, 0.0}
    },
    { // TAG 14
      {0.0, 0.0, 0.0}, {8.272272, 6.137656, 3.1415922 }, {0.0, 0.0, 0.0}
    },
    { // TAG 15  
      {0.0, 0.0, 0.0}, {8.272272, 1.914906, 3.1415922 }, {0.0, 0.0, 0.0}
    },
    { // TAG 16
      {0.0, 0.0, 0.0}, {5.987542, -0.00381, 1.5707961 }, {0.0, 0.0, 0.0}
    },
    { // TAG 17
      {3.9572184, 3.4317686, 0.0}, {4.073906, 3.306318, 4.1887896 }, {4.2411904, 3.2674306, 0.0}
    },
    { // TAG 18
      {3.7084,    4.190238, 0.0 }, {3.6576,   4.0259,   3.1415922 }, {3.7084,    3.861562,  0.0}
    },
    { // TAG 19    
      {4.2411904, 4.784471, 0.0}, {4.073906, 4.745482, 2.0943948 }, {3.9572184, 4.620133, 0.0}
    },
    { // TAG 20
      {5.0220372, 4.620133, 0.0 }, {4.90474,  4.745482, 1.0471974 }, {4.7374048, 4.784471,  0.0}
    },
    { // TAG 21   
      {5.269738,  3.861562, 0.0 }, {5.321046, 4.0259,   0.0       }, {5.269738,  4.190238,  0.0}
    },
    { // TAG 22
      {4.7374048, 3.2674306, 0.0}, {4.90474,  3.306318, 5.235987  }, {5.0214276, 3.4317686, 0.0}
    }
  };

  // EXAMPLE: TagPOse[1][RIGHT][X]

  int[] tagLocation;

  final NetworkTable table = NetworkTableInstance.getDefault().getTable("Limelight");

  public Limelight() {}

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

  public boolean zoned(){
    return ((tag > 11) ? Alliance.Blue : Alliance.Red) == DriverStation.getAlliance().get();
  }

  public boolean reefZone(){
    return (tag >=6 && tag <= 11 || tag >= 17 && tag <= 22);
  }
}
