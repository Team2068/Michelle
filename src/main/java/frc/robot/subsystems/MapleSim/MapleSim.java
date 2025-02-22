package frc.robot.subsystems.MapleSim;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralAlgaeStack;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MapleSim extends SubsystemBase {
        public static SimulatedArena Arena() {
                SimulatedArena Arena = SimulatedArena.getInstance();
                Arena.addGamePiece(new ReefscapeCoralOnField(
                                // We must specify a heading since the coral is a tube
                                new Pose2d(2, 2, Rotation2d.fromDegrees(90))));

                SimulatedArena.getInstance().addGamePiece(new ReefscapeAlgaeOnField(new Translation2d(2, 2)));
                SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralAlgaeStack(new Translation2d(2, 2)));

                return Arena;
        }

        StructArrayPublisher<Pose3d> Coral = NetworkTableInstance.getDefault().getTable("MapleSim")
                        .getStructArrayTopic("Coral", Pose3d.struct).publish();
        StructArrayPublisher<Pose3d> Algae = NetworkTableInstance.getDefault().getTable("MapleSim")
                        .getStructArrayTopic("Algae", Pose3d.struct).publish();

        @Override
        public void periodic() {
                Coral.set(MapleSim.Arena().getGamePiecesArrayByType("Coral"));
                Algae.set(MapleSim.Arena().getGamePiecesArrayByType("Algae"));
                
        }

}
