package frc.robot.swerve;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.Radians;

public class Swerve {

    public static final double PI2 = 2.0 * Math.PI;

    public static class Constants {
        // BOT SWITCHING
        public boolean comp = false;

        public double TRACKWIDTH = 19.5; // 30.0 for MKi
        public double WHEELBASE = 21.5; // 30.0 for MKi
        public double GEAR_RATIO;
        public double WHEEL_RADIUS;

        // DRIVER SETTINGS
        public int driver = 0;
        public double transFactor = .65; // factor = x/125, with x being the percentage of our max speed, same for the thing below
        public double rotFactor = .30; // .6 for tristan

        // AUTON CONSTANTS
        public double XControllerP = 3.35;
        public double XControllerD = 0.0;
        public double ThetaControllerP = 0;
        public double ThetaControllerD = 0;
        public RobotConfig autoConfig;
        public double MASS = 47;

        // BASE CHASSIS CONFIGURATION
        public static final double MAX_VELOCITY = 5.4;
        public static final String[] LAYOUT_TITLE = { "Front Left", "Front Right", "Back Left", "Back Right" };
        public static final int[] CHASSIS_ID = { 2, 3, 4, 5 }; // FL, FR, BL, BR
        public static final int[] ENCODER_ID = { 7, 8, 9, 10 }; // FL, FR, BL, BR
        public static double[] ENCODER_OFFSETS = {-0.8823, -0.8371, -0.6311, -0.7314};
        public static final int PIGEON_ID = 6;

        public Constants(){
            // compChassis = (System.getenv("BLUEBOT") == null); // Determine if we're comp based on an enviornmental variable like in 2023
            if (comp) {
                TRACKWIDTH = 30.0;
                WHEELBASE = 30.0;
                GEAR_RATIO = 8.14; // L1
                WHEEL_RADIUS = 0.1143;
                MASS = 60.0;
            } else {
                TRACKWIDTH = 19.5;
                WHEELBASE = 21.5;
                GEAR_RATIO = 6.12; // L3
                WHEEL_RADIUS = 0.1016;
                MASS = 47.0;
            }

            double trackwidthMeters = Units.inchesToMeters(TRACKWIDTH);
            double wheelbaseMeters = Units.inchesToMeters(WHEELBASE);
            Translation2d[] swerve_offsets =
                new Translation2d[] {
                    new Translation2d(trackwidthMeters / 2, wheelbaseMeters / 2),
                    new Translation2d(trackwidthMeters / 2, -wheelbaseMeters / 2),
                    new Translation2d(-trackwidthMeters / 2, wheelbaseMeters / 2),
                    new Translation2d(-trackwidthMeters / 2, -wheelbaseMeters / 2)
                };

            double moi = 1/12 * MASS * (trackwidthMeters * trackwidthMeters + wheelbaseMeters * wheelbaseMeters); // estimate of moi
            ModuleConfig moduleConfigs = new ModuleConfig(WHEEL_RADIUS, 5.4, 1.2, null, GEAR_RATIO, 59, 4);
            autoConfig = new RobotConfig(
                MASS,
                moi,
                moduleConfigs,
                swerve_offsets
            );

                        // try {
            //     autoConfig = RobotConfig.fromGUISettings();
            // } catch (IOException e) {
            //     e.printStackTrace();
            // } catch (ParseException e) {
            //     e.printStackTrace();
            // }

            switch (driver) {
                default: // Shaan
                transFactor = .65;
                rotFactor = .48;
                    break;
            }
        }
    }

    public interface Encoder {
        public void zero();
        public boolean connected();
        public double angle();
    }

    public static class Cancoder implements Encoder {

        CANcoder encoder;
        MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();

        public Cancoder(int id) {
            encoder = new CANcoder(id);
            magnetConfig.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
            magnetConfig.AbsoluteSensorDiscontinuityPoint = 1.0;
            magnetConfig.withMagnetOffset(Swerve.Constants.ENCODER_OFFSETS[id-7]);
            encoder.getConfigurator().apply(magnetConfig);
        }

        public void zero() {
            magnetConfig.MagnetOffset = -encoder.getAbsolutePosition().getValueAsDouble();
            encoder.getConfigurator().apply(magnetConfig);
        }

        public boolean connected(){
            return encoder.isConnected();
        }

        public double angle() {
            return ((encoder.getAbsolutePosition().getValue().in(Radians) + PI2) % PI2);
        }
    }

    public static class Canand implements Encoder {
        Canandmag encoder;

        public Canand(int id) {
            encoder = new Canandmag(id);
            CanandmagSettings settings = new CanandmagSettings();
            settings.setInvertDirection(true);
            encoder.clearStickyFaults();
            encoder.setSettings(settings);
        }

        public void zero() {
            encoder.setAbsPosition(0, 250);
        }

        public boolean connected(){
            return encoder.isConnected();
        }

        public double angle() {
            return (encoder.getAbsPosition() * PI2) % PI2;
        }
    }
}
