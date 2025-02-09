package frc.robot.utility;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;

public class IO extends SubsystemBase {
        public final Swerve chassis = new Swerve();
        // public final AlgaeIntake intake = new AlgaeIntake();
        // public final Elevator elevator = new Elevator();

        public CommandScheduler scheduler = CommandScheduler.getInstance();

        public IO() {
                DriverStation.silenceJoystickConnectionWarning(true);
        }

        // StructPublisher<Pose2d> estimated_pose = Util.table.getStructTopic("Estimated Pose", Pose2d.struct).publish();

        @Override
        public void periodic() {}
}