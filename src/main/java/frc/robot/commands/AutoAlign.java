// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utility.IO;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {
  IO io;

  public static final int LEFT = 0;
  public static final int CENTRE = 1;
  public static final int RIGHT = 2;

  public static final int X = 0;
  public static final int Y = 1;
  public static final int ROTATION = 2;
  public static final double[][][] TagPose = {
    // all units in meters and all rotations in degrees
    // {x, y, rotation}     {left pole}, {center tag}, {right pole}                           **origin of blue reef is (4.48932, 4.0259), origin of red reef is (13.05831, 4.0259)

    {{0.0,        0.0,       0.0}, {16.6972,  0.65532,  126}, {0.0,        0.0,       0.0}}, // TAG 1
    {{0.0,        0.0,       0.0}, {16.6972,  7.39648,  234}, {0.0,        0.0,       0.0}}, // TAG 2
    {{0.0,        0.0,       0.0}, {11.56081, 8.05561,  270}, {0.0,        0.0,       0.0}}, // TAG 3
    {{0.0,        0.0,       0.0}, {9.27608,  6.137656, 0.0}, {0.0,        0.0,       0.0}}, // TAG 4
    {{0.0,        0.0,       0.0}, {9.27608,  1.914906, 0.0}, {0.0,        0.0,       0.0}}, // TAG 5
    {{13.6319768, 2.7594306, 0.0}, {13.47445, 3.306318, 300}, {13.9166092, 2.9237686, 0.0}}, // TAG 6
    {{14.347698,  3.861562,  0.0}, {13.8905,  4.0259,   0.0}, {14.347698,  4.190238,  0.0}}, // TAG 7
    {{13.9166092, 5.128133,  0.0}, {13.47445, 4.745482, 60 }, {13.6319768, 5.292471,  0.0}}, // TAG 8
    {{12.4853192, 5.292471,  0.0}, {12.64336, 4.745482, 120}, {12.2006868, 5.128133,  0.0}}, // TAG 9
    {{11.769598,  4.190238,  0.0}, {12.22731, 4.0259,   180}, {11.769598,  3.861562,  0.0}}, // TAG 10
    {{12.2002296, 2.9247686, 0.0}, {12.64336, 3.306318, 240}, {12.4853192, 2.7594306, 0.0}}, // TAG 11
    {{0.0,        0.0,       0.0}, {0.851154, 0.65532,  54 }, {0.0,        0.0,       0.0}}, // TAG 12
    {{0.0,        0.0,       0.0}, {0.851154, 7.39648,  306}, {0.0,        0.0,       0.0}}, // TAG 13
    {{0.0,        0.0,       0.0}, {8.272272, 6.137656, 180}, {0.0,        0.0,       0.0}}, // TAG 14
    {{0.0,        0.0,       0.0}, {8.272272, 1.914906, 180}, {0.0,        0.0,       0.0}}, // TAG 15 
    {{0.0,        0.0,       0.0}, {5.987542, -0.00381, 90 }, {0.0,        0.0,       0.0}}, // TAG 16
    {{3.6320984,  2.9237686, 0.0}, {4.073906, 3.306318, 240}, {3.9160704,  2.7594306, 0.0}}, // TAG 17
    {{3.2004,     4.190238,  0.0}, {3.6576,   4.0259,   180}, {3.2004,     3.861562,  0.0}}, // TAG 18
    {{3.9160704,  5.292471,  0.0}, {4.073906, 4.745482, 120}, {3.6320984,  5.128133,  0.0}}, // TAG 19 
    {{5.3471572,  5.128133,  0.0}, {4.90474,  4.745482, 60 }, {5.0625248,  5.292471,  0.0}}, // TAG 20
    {{5.777738,   3.861562,  0.0}, {5.321046, 4.0259,   0.0}, {5.777738,   4.190238,  0.0}}, // TAG 21
    {{5.0625248,  2.7594306, 0.0}, {4.90474,  3.306318, 300}, {5.3465476,  2.9237686, 0.0}}  // TAG 22
  };

  // will eventually be seperatly adjusted
  PIDController pidY = new PIDController(0.10, 0.00, 0.00);
  PIDController pidX = new PIDController(0.10, 0.00, 0.00);
  PIDController pidR = new PIDController(0.10, 0.00, 0.00);

  public double poleX;
  public double poleY;
  public double tagRotation;
  int side;
  int tag;
  int zone;

  public AutoAlign(int side /** make sure that the number is either 0 or 2*/) {
    this.side = side;
    pidR.enableContinuousInput(0.0, 360.0); // CHECK IF THIS ACTUALLY ALLOWS WRAPPING
  }

  @Override
  public void initialize() {
    // find zone first
    Boolean cZone = DriverStation.getAlliance().get() == Alliance.Blue;
    zone = (cZone)? 
    (int) Math.atan((io.chassis.pose().getY()- 4.0259)/(io.chassis.pose().getX()-4.48932)): 
    (int) Math.atan((io.chassis.pose().getY()- 4.0259)/(io.chassis.pose().getX()-13.05831));
    zone = zone/60;

    switch (zone) {
      case 1:
        tag = (cZone)? 20: 8;
        break;
      case 2:
        tag = (cZone)? 19: 9;
        break;
      case 3:
        tag = (cZone)? 18: 10;
        break;
      case 4:
        tag = (cZone)? 17: 11;
        break;
      case 5:
        tag = (cZone)? 22: 6;
        break;
      default:
        tag = (cZone)? 21: 7;
        break;
    }
    // then check if the zone 

    // tag = zone
    // poleX = TagPose[tag][side][X];
    // poleY = TagPose[tag][side][Y];
    poleX = TagPose[tag - 1][side][X];
    poleY = TagPose[tag - 1][side][Y];
    tagRotation = TagPose[tag - 1][CENTRE][ROTATION];
  }

  @Override
  public void execute() {
    double currentRotation = io.chassis.getYaw();
    double currentPositionX = io.chassis.pose().getX();
    double currentPositionY = io.chassis.pose().getY();


    io.chassis.drive(new ChassisSpeeds(pidX.calculate(currentPositionX, poleX), pidY.calculate(currentPositionY, poleY), pidR.calculate(currentRotation, tagRotation)));
  }

  @Override
  public void end(boolean interrupted) {
    io.chassis.drive(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return (pidX.atSetpoint() && pidY.atSetpoint() && pidR.atSetpoint());
  }
}
