package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swerve.Module;
import frc.robot.swerve.Swerve.Constants;
import frc.robot.utility.LimelightHelpers;
import frc.robot.utility.Util;

public class Swerve extends SubsystemBase {

    public boolean field_oritented = true;
    private final SwerveDriveKinematics kinematics;

    public final Pigeon2 pigeon2 = new Pigeon2(Constants.PIGEON_ID);

    StructArrayPublisher<SwerveModuleState> current_states = Util.table
            .getStructArrayTopic("Current Module States", SwerveModuleState.struct).publish();
    StructArrayPublisher<SwerveModuleState> target_states = Util.table
            .getStructArrayTopic("Target Module States", SwerveModuleState.struct).publish();
    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getTable("Debug")
            .getStructTopic("Current pose", Pose2d.struct).publish();
    StructPublisher<Pose2d> estimatedPosePublisher = Util.table
            .getStructTopic("Estimated Pose", Pose2d.struct).publish();

    SwerveDrivePoseEstimator estimator;
 
    final SwerveDriveOdometry odometry;
    final Module[] modules = new Module[4];
    ChassisSpeeds speeds = new ChassisSpeeds();
    public final Constants constants = new Constants();

    public boolean active = true;

    public Swerve() {
        kinematics = new SwerveDriveKinematics(
                createTranslation(constants.TRACKWIDTH / 2.0, constants.WHEELBASE / 2.0),
                createTranslation(constants.TRACKWIDTH / 2.0, -constants.WHEELBASE / 2.0),
                createTranslation(-constants.TRACKWIDTH / 2.0, constants.WHEELBASE / 2.0),
                createTranslation(-constants.TRACKWIDTH / 2.0, -constants.WHEELBASE / 2.0));

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        for (int i = 0; i < modules.length; i++) {
            modules[i] = new Module(
                    tab.getLayout(Constants.LAYOUT_TITLE[i], BuiltInLayouts.kList)
                            .withSize(2, 4)
                            .withPosition(i * 2, 0),
                    Constants.CHASSIS_ID[i],
                    Constants.CHASSIS_ID[i],
                    Constants.ENCODER_ID[i],
                    constants.comp);
        }

        odometry = new SwerveDriveOdometry(kinematics, rotation(), modulePositions(),
                new Pose2d(0, 0, new Rotation2d()));

        estimator = new SwerveDrivePoseEstimator(kinematics, rotation(), modulePositions(), odometry.getPoseMeters());

        AutoBuilder.configure(
                this::pose,
                this::resetOdometry,
                () -> speeds,
                (speeds, feedforwards) -> drive(speeds),
                new PPHolonomicDriveController(
                        new PIDConstants(constants.XControllerP, 0.0, constants.XControllerD), // Translation PID
                        new PIDConstants(constants.ThetaControllerP, 0, constants.ThetaControllerD, 0.0) // Rotation PID
                ),
                constants.autoConfig,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                },
                this);
    }

    private Translation2d createTranslation(double x, double y) {
        return new Translation2d(x, y);
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

    private SwerveModulePosition modulePosition(Module module) {
        return new SwerveModulePosition(module.drivePosition(), Rotation2d.fromRadians(module.angle()));
    }

    private SwerveModuleState moduleState(Module module) {
        return new SwerveModuleState(module.velocity(), new Rotation2d(module.angle()));
    }

    public SwerveModulePosition[] modulePositions() {
        SwerveModulePosition[] pos = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++)
            pos[i] = modulePosition(modules[i]);
        return pos;
    }

    public double getYaw(){
        return pigeon2.getYaw().getValueAsDouble();
    }

    public SwerveModuleState[] moduleStates(Module[] modules) {
        SwerveModuleState[] state = new SwerveModuleState[4];
        for (int i = 0; i < modules.length; i++)
            state[i] = moduleState(modules[i]);
        return state;
    }

    public void adjustRotation() {
        pigeon2.setYaw((absoluteRotation() + 180) % 360);
    }

    public Pose2d pose() {
        return odometry.getPoseMeters();
        // return estimatePose();
    }

    public void resetOdometry() {
        resetOdometry(new Pose2d());
    }

    public void resetOdometry(Pose2d pose) {
        zeroGyro();
        resetPosition();

        odometry.resetPosition(rotation(), modulePositions(), pose);
    }

    public void resetPosition() {
        for (Module mod : modules)
            mod.resetDrivePosition();
    }

    public void syncEncoders() {
        for (Module mod : modules)
            mod.syncEncoders();
    }

    public void zeroAbsolute() {
        for (Module mod : modules)
            mod.zeroAbsolute();
    }

    public void resetSteerPositions() {
        for (Module mod : modules)
            mod.set(0, 0);
    }

    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.MAX_VELOCITY);
        for (int i = 0; i < modules.length; i++) {
            states[i].optimize(new Rotation2d(modules[i].angle()));
            modules[i].set((states[i].speedMetersPerSecond / Constants.MAX_VELOCITY) * .8,
                    states[i].angle.getRadians());
        }
    }

    public ChassisSpeeds getSpeeds() {
        return speeds;
    }

    public double getRoll() {
        return pigeon2.getRoll().getValueAsDouble();
    }

    public void toggle() {
        active = !active;

        if (!active) {
            for (Module mod : modules)
                mod.stop();
        }
    }

    public Pose2d estimatePose() {
        LimelightHelpers.SetRobotOrientation("limelight-main", absoluteRotation(), 0,0,0,0,0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-main");
        if (mt2 == null) return new Pose2d();
        if (!(Math.abs(pigeon2.getAngularVelocityXWorld().getValueAsDouble()) > 720|| mt2.tagCount == 0)){
            estimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
            estimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
            estimatedPosePublisher.set(mt2.pose);
        }
        return mt2.pose;
    }

    public void periodic() {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        if (active && speeds != new ChassisSpeeds())
            setModuleStates(states);

        current_states.set(moduleStates(modules));
        target_states.set(states);

        Pose2d pose = odometry.update(rotation(), modulePositions());
        estimatePose();
        posePublisher.set(pose);


        DogLog.log("Swerve/Current States", moduleStates(modules));
        DogLog.log("Swerve/Target States", states);

        DogLog.log("Swerve/X Position", pose.getX());
        DogLog.log("Swerve/Y Position", pose.getY());
        DogLog.log("Swerve/Pose", pose);
        DogLog.log("Swerve/Odometry Rotation", rotation().getDegrees());

        DogLog.log("Swerve/Pigeon Yaw", pigeon2.getYaw().getValueAsDouble());
        DogLog.log("Swerve/Pigeon Pitch", pigeon2.getPitch().getValueAsDouble());
        DogLog.log("Swerve/Pigeon Roll", pigeon2.getRoll().getValueAsDouble());
        DogLog.log("Swerve/Drive Mode", (field_oritented) ? "Field-Oriented" : "Robot-Oriented");
    }
}