package frc.robot.subsystems.Simulation;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final int kMotorPort = 0;
    public static final int kEncoderAChannel = 0;
    public static final int kEncoderBChannel = 1;
    // public static final int kJoystickPort = 0;
  
    public static final double  kElevatorKp= 8;
    public static final double kElevatorKi = 0.0;
    public static final double kElevatorKd = 2.0;
  
    // public static final double kElevatorkS = 0.0; // volts (V)
    // public static final double kElevatorkG = 0.762; // volts (V)
    // public static final double kElevatorkV = 0.762; // volt per velocity (V/(m/s))
    // public static final double kElevatorkA = 0.0; // volt per acceleration (V/(m/s²))
  
    public static final double kElevatorGearing = 13.56;
    public static final double kElevatorDrumRadius = Units.inchesToMeters(1.0);
    public static final double kCarriageMass = 11.0; // kg
  
    public static final double kSetpointMeters = 1.5;
    // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
    public static final double kMinElevatorHeightMeters = Units.inchesToMeters(38);
    public static final double kMaxElevatorHeightMeters = Units.inchesToMeters(78);
  
    // distance per pulse = (distance per revolution) / (pulses per revolution)
    //  = (Pi * D) / ppr
    public static final double kElevatorEncoderDistPerPulse =
        2.0 * Math.PI * kElevatorDrumRadius / 4096;
}
