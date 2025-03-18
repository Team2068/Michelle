package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utility.IO;

public class RotateChassis extends Command {

  IO io;
  double targetAngle;
  boolean automated = false;
  Translation2d reef = null;
  final PIDController rotationPID = new PIDController(0.01, 0.0, 0.001);

  public RotateChassis(IO io, double targetAngle) {
    this.io = io;
    this.targetAngle = targetAngle;
    rotationPID.setTolerance(5);
    rotationPID.enableContinuousInput(0, 360);
    automated = false;
    addRequirements(io.chassis);
  }

  public RotateChassis(IO io){
    this(io, 0);  
    boolean blue = DriverStation.getAlliance().get() == Alliance.Blue;
    reef = (blue) ? new Translation2d(4.5, 
    4) : new Translation2d(13.25, 4) ;
    automated = true;
  }

  @Override
  public void initialize() {
    if (automated){
      Translation2d mPose = io.chassis.pose().getTranslation();
      double reefBotAngle = (Math.toDegrees(Math.atan2(mPose.getX()-reef.getX(), mPose.getY()-reef.getY())) + 540.0) % 360;

      SmartDashboard.putNumber("Reef to Bot Angle", reefBotAngle);
      
      switch ( ((int)reefBotAngle) / 60) {
        case 1: targetAngle = 240; //  -120
          break;
        case 2: targetAngle = 300; // -60
          break;
        case 3: targetAngle = 0; // 0
          break;
        case 4: targetAngle = 60; // 60
          break;
        case 5: targetAngle = 120; // 120
          break;
        default: targetAngle = 180; // 180
          break;
      }
    }
  }

  @Override
  public void execute() {
    io.chassis.drive(new ChassisSpeeds(0, 0, rotationPID.calculate(io.chassis.getYaw(), targetAngle)));
  }

  @Override
  public void end(boolean interrupted) {
    io.chassis.drive(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    // return Math.abs(rotationPID.getError()) < 1;
    return rotationPID.atSetpoint();
  }
}
