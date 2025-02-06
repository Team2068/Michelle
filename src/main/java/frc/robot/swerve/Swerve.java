package frc.robot.swerve;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;
import static edu.wpi.first.units.Units.Radians;

import java.io.IOException;

import org.json.simple.parser.ParseException;

public class Swerve {

    public static final double PI2 = 2.0 * Math.PI;

    public static class Constants {
        // BOT SWITCHING
        public boolean comp = false;

        public double TRACKWIDTH = 19.5; // 30.0 for MKi
        public double WHEELBASE = 21.5; // 30.0 for MKi
        public double GEAR_RATIO;

        // DRIVER SETTINGS
        public int driver = 0;
        public double transFactor = .8; // factor = x/125, with x being the percentage of our max speed, same for the thing below
        public double rotFactor = .48; // .6 for tristan

        // AUTON CONSTANTS
        public double XControllerP = 0.1;
        public double XControllerD = 0.0;
        public double ThetaControllerP = 0.05;
        public double ThetaControllerD = 0.005;
        public RobotConfig autoConfig;

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
            } else {
                TRACKWIDTH = 19.5;
                WHEELBASE = 21.5;
                GEAR_RATIO = 6.12; // L3
            }

            switch (driver) {
                default: // Shaan
                transFactor = .8;
                rotFactor = .48;
                    break;
            }
            
            try {
                autoConfig = RobotConfig.fromGUISettings();
            } catch (IOException e) {
                e.printStackTrace();
            } catch (ParseException e) {
                e.printStackTrace();
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
