package frc.robot.swerve;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Module {
    public final TalonFX drive;
    public final SparkMax steer;
    public final Swerve.Encoder encoder;

    double desiredAngle;

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double DRIVE_REDUCTION = (15.0 / 32.0) * (10.0 / 60.0);
    public static final double STEER_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final double DRIVE_CONVERSION_FACTOR = Math.PI * WHEEL_DIAMETER * DRIVE_REDUCTION;

    public Module(ShuffleboardLayout tab, int driveID, int steerID, int encoderID, boolean comp) {
        drive = new TalonFX(driveID, "rio");
        steer = new SparkMax(steerID, MotorType.kBrushless);
        encoder = (comp) ? new Swerve.Cancoder(encoderID) : new Swerve.Canand(encoderID);

        SparkMaxConfig steerConfig = new SparkMaxConfig();

        steerConfig
                .smartCurrentLimit(20)
                .idleMode(IdleMode.kCoast)
                .inverted(false);

        steerConfig.encoder
                .positionConversionFactor(2 * Math.PI * STEER_REDUCTION)
                .velocityConversionFactor(Math.PI * STEER_REDUCTION / 60);

        steerConfig.closedLoop
                .positionWrappingEnabled(true)
                .positionWrappingMaxInput(Swerve.PI2)
                // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.2, 0.0, 0.0);

                steerConfig.signals.primaryEncoderPositionAlwaysOn(false);
                steerConfig.signals.primaryEncoderPositionPeriodMs(20);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 80;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.CurrentLimits.SupplyCurrentLimit = 20;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        steer.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        steer.getEncoder().setPosition(angle());

        drive.getConfigurator().apply(config);

        tab.addDouble("Absolute Angle", () -> Math.toDegrees(angle()));
        // tab.addDouble("Current Angle", () -> Math.toDegrees(steer.getEncoder().getPosition()));
        tab.addDouble("Angle Difference", () -> Math.toDegrees(angle() - steer.getEncoder().getPosition()));
        tab.addDouble("Target Angle", () -> Math.toDegrees(desiredAngle));
        tab.addBoolean("Active", encoder::connected);
    }

    public void resetDrivePosition() {
        drive.setPosition(0.0);
    }

    public void syncencoders() {
        steer.getEncoder().setPosition(encoder.angle());
    }

    public void zeroAbsolute() {
        encoder.zero();
    }

    public double drivePosition() {
        return drive.getPosition().getValueAsDouble() * .502 * WHEEL_DIAMETER;
    }

    public double velocity() {
        return drive.getRotorVelocity().getValueAsDouble() * Swerve.PI2 * .502 * Units.inchesToMeters(3);
    }

    public double angle() {
        return encoder.angle();
    }

    public void stop() {
        drive.stopMotor();
        steer.stopMotor();
    }

    public void set(double driveVolts, double targetAngle) {
        syncencoders();

        drive.set(driveVolts);
        steer.getClosedLoopController().setReference(targetAngle, ControlType.kPosition);
    }

}
