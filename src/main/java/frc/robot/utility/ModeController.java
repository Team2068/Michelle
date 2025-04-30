package frc.robot.utility;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
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
        for (int i = 0; i < 5; i++){
            SmartDashboard.putData("Elevator Height L" + i, io.elevator.moveCommand(i));
        }
        
        for (int i = 0; i < 5; i++){
            SmartDashboard.putData("Shooter Angle L" + i, io.shooter.angleCommand(i));
        }

        SmartDashboard.putData("Toggle Hang", new InstantCommand(() -> io.hang.toggleHang(), io.hang));

        SmartDashboard.putData("Toggle Elevator Soft Limits", new InstantCommand(() -> io.elevator.toggleSoftLimits(), io.elevator));
        SmartDashboard.putData("Toggle Shooter Pivot Soft Limits", new InstantCommand(() -> io.shooter.toggleSoftLimits(), io.shooter));
        
        SmartDashboard.putData("Toggle Shooter Redundancy", new InstantCommand(() -> io.shooter.togglePivotRedundancy(), io.shooter));
        SmartDashboard.putData("Toggle Elevator Redundancy", new InstantCommand(() -> io.elevator.toggleRedundancy(), io.elevator));
        SmartDashboard.putData("Toggle Encoder Redundancy", new InstantCommand(() -> io.chassis.toggleEncoderRedundancy(), io.chassis));
        SmartDashboard.putData("Toggle Module Redundancy", new InstantCommand(() -> io.chassis.toggleModuleRedundancy(), io.chassis));
        
        // TODO: Setup a toggle for Coral Detection Redundancy
        // TODO: Setup a toggle for Algae Detection Redundancy

        double elevatorkP = (double) Util.get("Elevator kP", 0.3);
        double elevatorkI = (double) Util.get("Elevator kI", 0.0);
        double elevatorkD = (double) Util.get("Elevator kD", 0.1);
        double elevatorkG = (double) Util.get("Elevator kG", 0.0);

        double shooterkP = (double) Util.get("Elevator kP", 0.3);
        double shooterkI = (double) Util.get("Elevator kI", 0.0);
        double shooterkD = (double) Util.get("Elevator kD", 0.1);
        
        double hangkP = (double) Util.get("Elevator kP", 0.3);
        double hangkI = (double) Util.get("Elevator kI", 0.0);
        double hangkD = (double) Util.get("Elevator kD", 0.1);

        SmartDashboard.putData("Set Elevator PID", new InstantCommand(() -> io.elevator.PID(elevatorkP, elevatorkI, elevatorkD, elevatorkG), io.elevator));
        SmartDashboard.putData("Set Shooter Pivot PID", new InstantCommand(() -> io.shooter.pivotPID(shooterkD, shooterkI, shooterkP), io.shooter));
        SmartDashboard.putData("Set Hang PID", new InstantCommand(() -> io.hang.PID(hangkP, hangkI, hangkD), io.hang));
    }
}
