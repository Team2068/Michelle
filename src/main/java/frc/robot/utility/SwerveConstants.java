package frc.robot.utility;

import edu.wpi.first.math.util.Units;

public class SwerveConstants {
    boolean compChassis = false;

    public static double TRACKWIDTH;
    public static double WHEELBASE;

    public static double GEAR_RATIO;

    public static final double MAX_VOLTAGE = 10;
    public static final double MAX_VELOCITY = 5;

    public static final String[] LAYOUT_TITLE = { "Front Left", "Front Right", "Back Left", "Back Right" };

    public static final int[] CHASSIS_ID = { 2, 3, 4, 5 }; // FL, FR, BL, BR
    public static final int[] ENCODER_ID = { 7, 8, 9, 10 }; // FL, FR, BL, BR

    public static final int PIGEON_ID = 6;

    public static final int TURBO = 1;
    public static final int SLOW = 2;

    public SwerveConstants() {
        if (compChassis){
            TRACKWIDTH = Units.inchesToMeters(30);
            WHEELBASE = Units.inchesToMeters(30);
            GEAR_RATIO = 6.12;
        } else {
            TRACKWIDTH = Units.inchesToMeters(19.5);
            WHEELBASE = Units.inchesToMeters(21.5);
            GEAR_RATIO = 8.14;
        }
    }
}
