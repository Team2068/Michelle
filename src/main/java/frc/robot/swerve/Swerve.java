package frc.robot.swerve;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;
import static edu.wpi.first.units.Units.Radians;

public class Swerve {

    public static final double PI2 = 2.0 * Math.PI;

    public interface Encoder {
        public void zero();
        public boolean connected();
        public double angle();
    }

    public static class Constants {
        boolean compChassis = false;

        public double TRACKWIDTH = 19.5; // 30.0 for MKi
        public double WHEELBASE = 21.5; // 30.0 for MKi
        public double GEAR_RATIO;
        public static final double MAX_VELOCITY = 5.4;
        public static final String[] LAYOUT_TITLE = { "Front Left", "Front Right", "Back Left", "Back Right" };
        public static final int[] CHASSIS_ID = { 2, 3, 4, 5 }; // FL, FR, BL, BR
        public static final int[] ENCODER_ID = { 7, 8, 9, 10 }; // FL, FR, BL, BR
        public static final int PIGEON_ID = 6;

        public Constants(){
            // compChassis = (System.getenv("BLUEBOT") == null); // Determine if we're comp based on an enviornmental variable like in 2023
            if (compChassis) {
                TRACKWIDTH = 30.0;
                WHEELBASE = 30.0;
                GEAR_RATIO = 8.14; // L1
            } else {
                TRACKWIDTH = 19.5;
                WHEELBASE = 21.5;
                GEAR_RATIO = 6.12; // L3
            }
        }
    }

    public static class Cancoder implements Encoder {

        CANcoder encoder;
        MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();

        public Cancoder(int id) {
            encoder = new CANcoder(id);
            magnetConfig.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
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
            Encoder.setSettings(settings);
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
