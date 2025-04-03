package frc.robot.utility;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ClearAlgae;
import frc.robot.commands.Intake;
import frc.robot.commands.ScoreReef;

public class ModeController {
    IO io;
    public final CommandGenericHID controller;
    public final SendableChooser<Runnable> selector = new SendableChooser<Runnable>();

    int mode = 0;

    final int AUTOMATED = 0;
    final int MANUAL = 1;
    final int DEMO = 2;

    final int CHARACTERISATION = 3;
    final int DEBUG = 4;

    enum povDirection {
        Up,
        Down,

        Left,
        LeftUp,
        LeftDown,

        Right,
        RightUp,
        RightDown,

        Centre,
    }

    final int[] directions = {0, 180, 270, 315, 225, 90, 45, 135, -1};
    

    public ModeController(IO io, int port){
        this.io = io;
        controller = new CommandGenericHID(port);
        selector.setDefaultOption("Automated", () -> {mode = 0;});
        selector.addOption("Manual", () -> {mode = 1;});
        selector.addOption("Characterise", () -> {mode = 2;});
        selector.addOption("Debug", () -> {mode = 3;});
        selector.onChange((x) -> {x.run();});

        configureGeneral();
        configureAutomated();
        configureManual();
        configureCharacterisaton();
        configureDebug();
    }

    public void toggleMode(){
        mode = (mode + 1 % 3);
    }

    public Trigger binding(Button button, int targetMode, BooleanSupplier... conditions){
        var binding = controller.button(button.value).and(() -> (mode == targetMode));
        for (int i = 0; i < conditions.length; i++)
            binding.and(conditions[i]);
        return binding;
    }

    public Trigger binding(Axis axis, int targetMode, BooleanSupplier... conditions){
        var binding = controller.axisMagnitudeGreaterThan(axis.value, 0.2).and(() -> (mode == targetMode));
        for (int i = 0; i < conditions.length; i++)
            binding.and(conditions[i]);
        return binding;
    }

    public Trigger binding(povDirection direction, int targetMode, BooleanSupplier... conditions){
        var binding = controller.pov(directions[direction.ordinal()]).and(() -> (mode == targetMode));
        for (int i = 0; i < conditions.length; i++)
            binding.and(conditions[i]);
        return binding;
    }

    public void configureGeneral(){
        controller.button(Button.kBack.value).onTrue(Util.Do(io.chassis::resetAngle, io.chassis));
    }

    public void configureAutomated(){
        IntSupplier pos = () -> { return ((controller.getHID().getRawButtonPressed(Button.kLeftBumper.value)) ? -1 : 0) + ((controller.getHID().getRawButtonPressed(Button.kRightBumper.value)) ? 1 : 0) + 1; };
        binding(Button.kY, AUTOMATED, () -> io.shooter.coral()).onTrue(new ScoreReef(io, pos.getAsInt(), 4));
        binding(Button.kX, AUTOMATED, () -> io.shooter.coral()).onTrue(new ScoreReef(io, pos.getAsInt(), 3));
        binding(Button.kB, AUTOMATED, () -> io.shooter.coral()).onTrue(new ScoreReef(io, pos.getAsInt(), 2));
        binding(Button.kA, AUTOMATED, () -> io.shooter.coral()).onTrue(new ScoreReef(io, pos.getAsInt(), 1));
        binding(Button.kA, AUTOMATED, () -> !io.shooter.coral()).onTrue(new Intake(io, false));
        binding(Button.kStart, AUTOMATED).onTrue(Util.Do(() -> io.elevator.move(0)));

        // binding(povDirection.Up, AUTOMATED, () -> !io.shooter.algae()).onTrue(new ClearAlgae(io));
        // binding(povDirection.Up, AUTOMATED, () -> io.shooter.algae()).onTrue(Util.Do(() ->io.shooter.speed(1), io.shooter)).onFalse(Util.Do(() -> {
        //     io.shooter.stopIntake();
        //     io.shooter.angle(0);
        //     io.elevator.move(0);
        // }, io.shooter));

        // // TODO: Check if this works as expected
        // binding(povDirection.Left, AUTOMATED, () -> !io.shooter.algae()).toggleOnTrue(Util.Do(() -> {
        //     io.shooter.speed(-1);
        //     io.shooter.angle(0);
        // }, io.shooter)).toggleOnFalse(Util.Do(() -> {
        //     io.shooter.stopIntake();
        //     io.shooter.angle(0);
        // }, io.shooter));
        // binding(povDirection.Left, AUTOMATED, () -> io.shooter.algae()).onTrue(io.elevator.moveCommand(5));

        binding(povDirection.Down,AUTOMATED).onTrue(Util.Do(io.hang::toggleHang, io.hang));
    }

    public void configureManual(){
        binding(Button.kBack, MANUAL).onTrue(Util.Do(io.chassis::resetOdometry, io.chassis));
        binding(Button.kLeftBumper, MANUAL).onTrue(Util.Do(() ->io.elevator.volts(-4), io.elevator)).onFalse(Util.Do(io.elevator::stop, io.elevator));
        binding(Button.kRightBumper, MANUAL).onTrue(Util.Do(() ->io.elevator.volts(4), io.elevator)).onFalse(Util.Do(io.elevator::stop, io.elevator));

        // TODO: Check if this is safe
        binding(Axis.kLeftTrigger, MANUAL).whileTrue(Util.Do(() -> io.shooter.speed(controller.getRawAxis(Axis.kLeftTrigger.value))));
        binding(Axis.kRightTrigger, MANUAL).whileTrue(Util.Do(() -> io.shooter.pivotSpeed(controller.getRawAxis(Axis.kRightTrigger.value))));

        binding(Button.kY, MANUAL).onTrue(Util.Do(() -> io.shooter.speed(-1), io.shooter)).onFalse(Util.Do(io.shooter::stopIntake, io.shooter));
        binding(Button.kX, MANUAL).onTrue(Util.Do(() -> io.shooter.hoodSpeed(.1), io.shooter)).onFalse(Util.Do(() -> io.shooter.hoodSpeed(0), io.shooter));
        binding(Button.kA, MANUAL).onTrue(Util.Do(() -> io.shooter.hoodSpeed(-.1), io.shooter)).onFalse(Util.Do(() -> io.shooter.hoodSpeed(0), io.shooter));

        binding(povDirection.Down, MANUAL).onTrue(Util.Do(io.chassis::toggle));
        binding(povDirection.Left, MANUAL).onTrue(Util.Do(io.chassis::syncEncoders));
        binding(povDirection.Right, MANUAL, () -> !io.chassis.active).onTrue(Util.Do(io.chassis::zeroAbsolute)); // TODO: Add rumble effect
    }

    boolean mechanismMode = false;

    public void configureCharacterisaton(){
        binding(Button.kStart, CHARACTERISATION).onTrue(Util.Do(() -> mechanismMode = !mechanismMode));

        binding(Button.kX, CHARACTERISATION, () -> !mechanismMode).toggleOnTrue(io.chassis.driveRoutine.quasistatic(Direction.kForward));
        binding(Button.kA, CHARACTERISATION, () -> !mechanismMode).toggleOnTrue(io.chassis.driveRoutine.quasistatic(Direction.kReverse));
        binding(Button.kY, CHARACTERISATION, () -> !mechanismMode).toggleOnTrue(io.chassis.driveRoutine.dynamic(Direction.kForward));
        binding(Button.kB, CHARACTERISATION, () -> !mechanismMode).toggleOnTrue(io.chassis.driveRoutine.dynamic(Direction.kReverse));

        binding(povDirection.Left, CHARACTERISATION, () -> !mechanismMode).toggleOnTrue(io.chassis.steerRoutine.quasistatic(Direction.kForward));
        binding(povDirection.Down, CHARACTERISATION, () -> !mechanismMode).toggleOnTrue(io.chassis.steerRoutine.quasistatic(Direction.kReverse));
        binding(povDirection.Right, CHARACTERISATION, () -> !mechanismMode).toggleOnTrue(io.chassis.steerRoutine.dynamic(Direction.kReverse));
        binding(povDirection.Up, CHARACTERISATION, () -> !mechanismMode).toggleOnTrue(io.chassis.steerRoutine.dynamic(Direction.kForward));
        
        binding(Button.kX, CHARACTERISATION, () -> mechanismMode).toggleOnTrue(io.shooter.pivotRoutine.quasistatic(Direction.kForward));
        binding(Button.kA, CHARACTERISATION, () -> mechanismMode).toggleOnTrue(io.shooter.pivotRoutine.quasistatic(Direction.kReverse));
        binding(Button.kY, CHARACTERISATION, () -> mechanismMode).toggleOnTrue(io.shooter.pivotRoutine.dynamic(Direction.kForward));
        binding(Button.kB, CHARACTERISATION, () -> mechanismMode).toggleOnTrue(io.shooter.pivotRoutine.dynamic(Direction.kReverse));

        binding(povDirection.Left, CHARACTERISATION, () -> mechanismMode).toggleOnTrue(io.elevator.routine.quasistatic(Direction.kForward));
        binding(povDirection.Down, CHARACTERISATION, () -> mechanismMode).toggleOnTrue(io.elevator.routine.quasistatic(Direction.kReverse));
        binding(povDirection.Up, CHARACTERISATION, () -> mechanismMode).toggleOnTrue(io.elevator.routine.dynamic(Direction.kForward));
        binding(povDirection.Right, CHARACTERISATION, () -> mechanismMode).toggleOnTrue(io.elevator.routine.dynamic(Direction.kReverse));
    }

    public void configureDebug(){
        // TODO: Setup Smart Dashboard buttons to pass in the Level for the elevator
        // TODO: Setup a shuffleboard thing to set the claw Angle based on Level (use the adjustable for the angles)
        // TODO: Setup a Shuffleboard thing to move the bot to particular States (Ground intake, no Algae, etc)
        // TODO: Setup Manual Movement of key mechanisms
        // TODO: Setup a toggle for Softlimits on Elevator Height, Shooter Angle, and Maybe Hang
        
        // TODO: Setup a toggle for Shooter Pivot Redundancy
        // TODO: Setup a toggle for Coral Detection Redundancy
        // TODO: Setup a toggle for Algae Detection Redundancy
        // TODO: Setup a toggle for Elevator Redundancy
        // TODO: Setup a toggle for Module Encoder Redundancy
        // TODO: Setup a toggle for Module Redundancy / Redundant Kinemtics

        // TODO: Setup Adjustable Height Locations for each level
        // TODO: Setup Adjustable Shooter Angles for each Level
        // TODO: Setup adjustable Hood angles for each Level
        // TODO: Setup adjustable Hang Positions
        
        // TODO: Setup Adjustable PID for Elevator
        // TODO: Setup Adjustable PID for Shooter Pivot
        // TODO: Maybe add a PID for target RPM
        // TODO: Setup Adjustable PID for Hang
    }
}
