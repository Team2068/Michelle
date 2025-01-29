package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.modules.KrakenSwerveModule;
import frc.robot.utility.SwerveConstants;
import frc.robot.utility.Util;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

public class Swerve extends SubsystemBase {

    public boolean field_oritented = true;
    public boolean slow_mode = false;
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(SwerveConstants.TRACKWIDTH / 2.0,
                    SwerveConstants.WHEELBASE / 2.0),
            new Translation2d(SwerveConstants.TRACKWIDTH / 2.0,
                    -SwerveConstants.WHEELBASE / 2.0),
            new Translation2d(-SwerveConstants.TRACKWIDTH / 2.0,
                    SwerveConstants.WHEELBASE / 2.0),
            new Translation2d(-SwerveConstants.TRACKWIDTH / 2.0,
                    -SwerveConstants.WHEELBASE / 2.0));

    public final Pigeon2 pigeon2 = new Pigeon2(SwerveConstants.PIGEON_ID);
    public final Timer syncTimer = new Timer();

    StructArrayPublisher<SwerveModuleState> current_states = Util.table
            .getStructArrayTopic("Current Module States", SwerveModuleState.struct).publish();
    StructArrayPublisher<SwerveModuleState> target_states = Util.table
            .getStructArrayTopic("Target Module States", SwerveModuleState.struct).publish();
    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getTable("Debug")
            .getStructTopic("Current pose", Pose2d.struct).publish();

    final SwerveDriveOdometry odometry;
    final KrakenSwerveModule[] modules = new KrakenSwerveModule[4];
    ChassisSpeeds speeds = new ChassisSpeeds();

    public boolean active = true;

    public Swerve() {
        new SwerveConstants();
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        for (int i = 0; i < modules.length; i++) {
            modules[i] = new KrakenSwerveModule(
                    tab.getLayout(SwerveConstants.LAYOUT_TITLE[i], BuiltInLayouts.kList)
                            .withSize(2, 4)
                            .withPosition(i * 2, 0),
                    SwerveConstants.CHASSIS_ID[i],
                    SwerveConstants.CHASSIS_ID[i],
                    SwerveConstants.ENCODER_ID[i]);
        }

        odometry = new SwerveDriveOdometry(kinematics, rotation(), modulePositions(),
                new Pose2d(0, 0, new Rotation2d()));
        syncTimer.start();
    }

    public void zeroGyro() {
        pigeon2.setYaw(0);
    }

    public Rotation2d rotation() {
        return new Rotation2d(absoluteRotation());
    }

    public double absoluteRotation() {
        double rotation = pigeon2.getYaw().getValueAsDouble() % 360;
        rotation += (rotation < 0) ? 360 : 0;
        return Math.toRadians(rotation);
    }

    public void drive(ChassisSpeeds speeds) {
        this.speeds = speeds;
    }

    public void stop() {
        speeds = new ChassisSpeeds();
    }

    public double distance(Pose2d reference_point) {
        return odometry.getPoseMeters().getTranslation().getDistance(reference_point.getTranslation());
    }

    public double distance(double[] reference_point) {
        return distance(new Pose2d(reference_point[0], reference_point[2], new Rotation2d(reference_point[3])));
    }

    private SwerveModulePosition modulePosition(KrakenSwerveModule module) {
        return new SwerveModulePosition(module.drivePosition(), Rotation2d.fromRadians(module.angle()));
    }

    private SwerveModuleState moduleState(KrakenSwerveModule module) {
        return new SwerveModuleState(module.velocity(), new Rotation2d(module.angle()));
    }

    public SwerveModulePosition[] modulePositions() {
        SwerveModulePosition[] pos = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++)
            pos[i] = modulePosition(modules[i]);
        return pos;
    }

    public SwerveModuleState[] moduleStates(KrakenSwerveModule[] modules) {
        SwerveModuleState[] state = new SwerveModuleState[4];
        for (int i = 0; i < modules.length; i++)
            state[i] = moduleState(modules[i]);
        return state;
    }

    public void adjustRotation() {
        double rotation = (absoluteRotation() - 180) % 360;
        rotation += (rotation < 0) ? 360 : 0;
        pigeon2.setYaw(rotation);
    }

    public Pose2d pose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry() {
        setOdometry(new Pose2d());
    }

    public void resetOdometry(Pose2d pose) {
        pigeon2.setYaw(0);
        resetPosition();

        odometry.resetPosition(rotation(), modulePositions(), pose);
        odometry.resetPosition(rotation(), modulePositions(), pose);
    }

    public void setOdometry(Pose2d pose) {
        zeroGyro();
        resetPosition();

        odometry.resetPosition(rotation(), modulePositions(), pose);
        odometry.resetPosition(rotation(), modulePositions(), pose);
    }

    public void resetPosition() {
        for (KrakenSwerveModule mod : modules)
            mod.resetDrivePosition();
    }

    public void syncEncoders() {
        for (KrakenSwerveModule mod : modules)
            mod.syncSteerEncoders();
    }

    public void resetAbsolute() {
        for (KrakenSwerveModule mod : modules)
            mod.resetAbsolute();
    }

    public void resetSteerPositions() {
        for (KrakenSwerveModule mod : modules)
            mod.set(0, 0);
    }

    public void setModuleStates(SwerveModuleState[] states) { 
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_VELOCITY);
        for (int i = 0; i < modules.length; i++) {
            // // cosine compensation
            // states[i].speedMetersPerSecond = states[i].speedMetersPerSecond * Math.cos(states[i].angle.getRadians()-moduleStates(modules)[i].angle.getRadians());

            modules[i].set((states[i].speedMetersPerSecond / SwerveConstants.MAX_VELOCITY), states[i].angle.getRadians());
        }
    }

    public ChassisSpeeds getSpeeds() {
        return speeds;
    }

    public void enable() {
        active = true;
    }

    public void disable() {
        active = false;

        for (KrakenSwerveModule mod : modules)
            mod.stop();
    }

    public void periodic() {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        if (active && speeds != new ChassisSpeeds())
            setModuleStates(states);
        current_states.set(moduleStates(modules));
        target_states.set(states);
        Pose2d pose = odometry.update(rotation(), modulePositions());
        posePublisher.set(pose);

        // if (syncTimer.get() % 2 == 0)
        //     syncEncoders();

        SmartDashboard.putNumber("X position", pose.getX());
        SmartDashboard.putNumber("Y position", pose.getY());

        SmartDashboard.putNumber("Odometry rotation", rotation().getDegrees());
        SmartDashboard.putNumber("Pigeon Yaw", pigeon2.getYaw().getValueAsDouble());
        SmartDashboard.putNumber("Pigeon Pitch", pigeon2.getPitch().getValueAsDouble());
        SmartDashboard.putNumber("Pigeon Roll", pigeon2.getRoll().getValueAsDouble());

        SmartDashboard.putString("Drive Mode", (field_oritented) ? "Field-Oriented" : "Robot-Oriented");
    }
}