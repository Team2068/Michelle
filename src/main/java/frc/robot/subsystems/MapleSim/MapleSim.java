package frc.robot.subsystems.MapleSim;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralAlgaeStack;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.littletonrobotics.junction.Logger;


import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MapleSim extends SubsystemBase {
        
        // Create and configure a drivetrain simulation configuration
        final static DriveTrainSimulationConfig driveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
                // Specify gyro type (for realistic gyro drifting and error simulation)
                .withGyro(COTS.ofPigeon2())
                // Specify swerve module (for realistic swerve dynamics)
                .withSwerveModule(COTS.ofMark4(
                        DCMotor.getKrakenX60(1), // Drive motor is a Kraken X60
                        DCMotor.getFalcon500(1), // Steer motor is a Falcon 500
                        COTS.WHEELS.COLSONS.cof, // Use the COF for Colson Wheels
                        3)) // L3 Gear ratio
                // Configures the track length and track width (spacing between swerve modules)
                .withTrackLengthTrackWidth(Inches.of(24), Inches.of(24))
                // Configures the bumper size (dimensions of the robot bumper)
                .withBumperSize(Inches.of(30), Inches.of(30));
     
        /* Create a swerve drive simulation */
        static SwerveDriveSimulation swerve = new SwerveDriveSimulation(
                // Specify Configuration
                driveTrainSimulationConfig,
                // Specify starting pose
                new Pose2d(3, 3, new Rotation2d()));
                //  drive = new Drive(
                //         new GyroIOSim(swerve.getGyroSimulation()),
                //         new ModuleIOSim(swerve.getModules()[0]),
                //         new ModuleIOSim(swerve.getModules()[1]),
                //         new ModuleIOSim(swerve.getModules()[2]),
                //         new ModuleIOSim(swerve.getModules()[3])
                // );
                //TODO: Implement this.drive

                // Example gyro interface
public interface GyroIO {
        Rotation2d getGyroRotation();
        AngularVelocity getGyroAngularVelocity();
    }
        // Simulation implementation
public class GyroIOSim implements GyroIO {
    private final GyroSimulation gyroSimulation;
    public GyroIOSim(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;
    }

    @Override // specified by GroIOSim interface
    public Rotation2d getGyroRotation() {
        return this.gyroSimulation.getGyroReading();
    }

    @Override // specified by GroIOSim interface
    public AngularVelocity getGyroAngularVelocity() {
        return this.gyroSimulation.getMeasuredAngularVelocity();
    }
}
          // This is only an example simulation IO implement, please change the code according to your ModuleIO interface
          //TODO: Find ModuleIO Interface
public class ModuleIOSim {
    // reference to module simulation
    private final SwerveModuleSimulation moduleSimulation;
//     reference to the simulated drive motor
    private final SimulatedMotorController.GenericMotorController driveMotor;
    // reference to the simulated turn motor
    private final SimulatedMotorController.GenericMotorController turnMotor;

    public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
        this.moduleSimulation = moduleSimulation;

        // configures a generic motor controller for drive motor
        // set a current limit of 60 amps
        this.driveMotor = moduleSimulation
                .useGenericMotorControllerForDrive()
                .withCurrentLimit(Amps.of(60));
        this.turnMotor = moduleSimulation
                .useGenericControllerForSteer()
                .withCurrentLimit(Amps.of(20));
    }

     // specified by ModuleIO interface
    public void setDriveOutputVoltage(Voltage voltage) {
        this.driveMotor.requestVoltage(voltage);
    }

    // specified by ModuleIO interface
    public void setSteerOutputVoltage(Voltage voltage) {
        this.turnMotor.requestVoltage(voltage);
    }

    // specified by ModuleIO interface
    public Rotation2d getSteerFacing() {
        return this.moduleSimulation.getSteerAbsoluteFacing();
    }

    // specified by ModuleIO interface
    public Angle getSteerRelativePosition() {
        return moduleSimulation.getSteerRelativeEncoderPosition().div(1.0); //TODO: SwerveModuleSimulation.STEER_GEAR_RATIO
    }

    // specified by ModuleIO interface
    public Angle getDriveWheelrPositiond() {
        return moduleSimulation.getDriveWheelFinalPosition();
    }
}  


        public static SimulatedArena Arena() {
                SimulatedArena Arena = SimulatedArena.getInstance();
                Arena.addGamePiece(new ReefscapeCoralOnField(
                                // We must specify a heading since the coral is a tube
                                new Pose2d(2, 2, Rotation2d.fromDegrees(90))));

                Arena.addGamePiece(new ReefscapeAlgaeOnField(new Translation2d(2, 2)));
                Arena.addGamePiece(new ReefscapeCoralAlgaeStack(new Translation2d(2, 2)));
                
                // Register the drivetrain simulation to the default simulation world
                
                 Arena.addDriveTrainSimulation(swerve);
                return Arena;
        }

        // StructArrayPublisher<Pose3d> Coral =
        // NetworkTableInstance.getDefault().getTable("MapleSim")
        // .getStructArrayTopic("Coral", Pose3d.struct).publish();
        // StructArrayPublisher<Pose3d> Algae =
        // NetworkTableInstance.getDefault().getTable("MapleSim")
        // .getStructArrayTopic("Algae", Pose3d.struct).publish();
        SimulatedArena Arena = Arena();

        @Override
        public void periodic() {
                // Coral.set(MapleSim.Arena().getGamePiecesArrayByType("Coral"));
                // Algae.set(MapleSim.Arena().getGamePiecesArrayByType("Algae"));
                Logger.recordOutput("FieldSimulation/Algae",
                                Arena.getGamePiecesArrayByType("Algae"));
                Logger.recordOutput("FieldSimulation/Coral",
                                Arena.getGamePiecesArrayByType("Coral"));
        }

}
