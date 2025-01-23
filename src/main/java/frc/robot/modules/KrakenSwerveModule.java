package frc.robot.modules;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;

public class KrakenSwerveModule {
    public final TalonFX driveMotor;
    public final SparkMax steerMotor;
    public final Canandmag steerEncoder;

    double desiredAngle;

    final double PI2 = 2.0 * Math.PI;

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double DRIVE_REDUCTION = (15.0 / 32.0) * (10.0 / 60.0);
    public static final double STEER_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final double DRIVE_CONVERSION_FACTOR = Math.PI * WHEEL_DIAMETER * DRIVE_REDUCTION;

    public KrakenSwerveModule(ShuffleboardLayout tab, int driveID, int steerID, int steerCANID) {
        driveMotor = new TalonFX(driveID, "rio");
        steerMotor = new SparkMax(steerID, MotorType.kBrushless);
        steerEncoder = new Canandmag(steerCANID);

        SparkMaxConfig steerConfig = new SparkMaxConfig();

        steerConfig
                .smartCurrentLimit(20)
                .idleMode(IdleMode.kBrake)
                .inverted(true);

        steerConfig.encoder
                .positionConversionFactor(Math.PI * STEER_REDUCTION)
                .velocityConversionFactor(Math.PI * STEER_REDUCTION / 60);

        steerConfig.closedLoop
                .positionWrappingEnabled(true)
                .positionWrappingMaxInput(PI2)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.2, 0.0, 0.0);
                
        steerConfig.signals.primaryEncoderPositionPeriodMs(20);

        CanandmagSettings settings = new CanandmagSettings();
        settings.setInvertDirection(true);

        steerEncoder.clearStickyFaults();
        steerEncoder.setSettings(settings);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 80;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 20;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        steerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        steerMotor.getEncoder().setPosition(angle());

        driveMotor.getConfigurator().apply(config);

        tab.addDouble("Absolute Angle", () -> Math.toDegrees(angle()));
        tab.addDouble("Current Angle", () -> Math.toDegrees(steerMotor.getEncoder().getPosition()));
        tab.addDouble("Target Angle", () -> Math.toDegrees(desiredAngle));
        tab.addBoolean("Active", steerEncoder::isConnected);
    }

    public void resetDrivePosition() {
        driveMotor.setPosition(0.0);
    }

    public void syncSteerEncoders() {
        steerMotor.getEncoder().setPosition(angle());
    }

    public void resetAbsolute() {
        steerEncoder.setAbsPosition(0, 250);
    }

    public double drivePosition() {
        return driveMotor.getPosition().getValueAsDouble() * .502 * WHEEL_DIAMETER;
    }

    public double velocity() {
        return driveMotor.getRotorVelocity().getValueAsDouble() * PI2 * .502 * Units.inchesToMeters(3);
    }

    public double angle() {
        return (steerEncoder.getAbsPosition() * PI2) % PI2;
    }

    public void stop() {
        driveMotor.stopMotor();
        steerMotor.stopMotor();
    }

    public void set(double driveVolts, double targetAngle) {
        // syncSteerEncoders();

        targetAngle %= PI2;
        targetAngle += (targetAngle < 0.0) ? PI2 : 0.0;

        desiredAngle = targetAngle;

        double diff = targetAngle -    steerMotor.getEncoder().getPosition();

        if (diff > (Math.PI / 2.0) || diff < -(Math.PI / 2.0)) {
            targetAngle = (targetAngle + Math.PI) % PI2;
            driveVolts *= -1.0;
        }

        driveMotor.set(driveVolts);
        steerMotor.getClosedLoopController().setReference(targetAngle, ControlType.kPosition);
    }

}