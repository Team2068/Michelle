package frc.robot.subsystems.Simulation;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class MapleSim {
    public static SimulatedArena Arena() {
        SimulatedArena Arena = SimulatedArena.getInstance();
        Arena.addGamePiece(new ReefscapeCoralOnField(
    // We must specify a heading since the coral is a tube
    new Pose2d(2, 2, Rotation2d.fromDegrees(90))));

SimulatedArena.getInstance().addGamePiece(new ReefscapeAlgaeOnField(new Translation2d(2,2)));














return Arena;
    }
}
