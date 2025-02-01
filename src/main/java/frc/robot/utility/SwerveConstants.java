package frc.robot.utility;

public class SwerveConstants {
    boolean compChassis = false;

    public static double TRACKWIDTH = 19.5; // 30.0 for MKi
    public static double WHEELBASE = 21.5;  // 30.0 for MKi

    public static double GEAR_RATIO;

    public static final double MAX_VELOCITY = 5.4;

    public static final String[] LAYOUT_TITLE = { "Front Left", "Front Right", "Back Left", "Back Right" };

    public static final int[] CHASSIS_ID = { 2, 3, 4, 5 };  // FL, FR, BL, BR
    public static final int[] ENCODER_ID = { 7, 8, 9, 10 }; // FL, FR, BL, BR

    public static final int PIGEON_ID = 6;

}