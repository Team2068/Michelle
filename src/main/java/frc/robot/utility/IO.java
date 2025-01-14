package frc.robot.utility;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class IO extends SubsystemBase {
        public final AutomatedController main = new AutomatedController(0, this);
        public final AutomatedController backup = new AutomatedController(1, this);

        public final Swerve chassis = new Swerve();
        public final AlgaeIntake intake = new AlgaeIntake();
        public final Elevator elevator = new Elevator();

        public CommandScheduler scheduler = CommandScheduler.getInstance();

        public IO() {
                SmartDashboard.putData("Main-Controller Mode", main.selector);
                SmartDashboard.putData("Backup-Controller Mode", main.selector);

                DriverStation.silenceJoystickConnectionWarning(true);
                chassis.setDefaultCommand(new DefaultDrive(this, main.controller));
        }

        // StructPublisher<Pose2d> estimated_pose = Util.table.getStructTopic("Estimated Pose", Pose2d.struct).publish();

        @Override
        public void periodic() {}
}