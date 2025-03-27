package frc.robot.utility;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Intake;
import frc.robot.commands.ScoreReef;

public class AutomatedController {
    public final CommandXboxController controller;
    public final SendableChooser<Runnable> selector = new SendableChooser<Runnable>();
    public int mode = 3;
    public GenericHID rumble = new GenericHID(0);

    IO io;

    public AutomatedController(int port, IO io){
        this.io = io;

        selector.setDefaultOption("Automated", () -> {mode = 0;});
        selector.addOption("Manual", () -> {mode = 1;});
        selector.addOption("Characterise", () -> {mode = 2;});
        selector.addOption("Debug", () -> {mode = 3;});

        selector.onChange((x) -> {x.run();});

        controller = new CommandXboxController(port);
        // rumble = new Rumble(controller.getHID());

        controller.rightStick().onTrue(new InstantCommand(() -> io.chassis.field_oritented = !io.chassis.field_oritented));
        controller.leftStick().debounce(2).onTrue(new InstantCommand(io.chassis::resetAngle));
        configure();
        // controller.back().onTrue(Util.Do(io.elevator::rest));
        // controller.start().onTrue(Util.Do(io.elevator::zero));

    }

    public BooleanSupplier mode(int targetMode){
        return () -> {return mode == targetMode;};
    }

    public BooleanSupplier automated(){
        return mode(0);
    }
    
    public BooleanSupplier manual(){
        return mode(1);
    }

    public BooleanSupplier characterise(){
        return mode(2);
    }

    public BooleanSupplier debug(){
        return mode(3);
    }

    public void switchMode(){
        mode = (mode + 1) % 3;
    }

    public void configure(){
        controller.start().and(controller.getHID()::getBackButtonPressed).onTrue(Util.Do(this::switchMode));
        configureAutomated();
        configureManual();
        configureCharacterisaton();
        configureDebug();
    }

    void configureAutomated(){

        IntSupplier pos = () -> { return ((controller.getHID().getLeftBumperButtonPressed()) ? -1 : 0) + ((controller.getHID().getLeftBumperButtonPressed()) ? 1 : 0) + 1; };

        controller.y().and(automated()).and(() -> io.shooter.hasCoral()).onTrue(Util.Do(() -> new ScoreReef(io, pos.getAsInt() ,4)));
        controller.x().and(automated()).and(() -> io.shooter.hasCoral()).onTrue(Util.Do(() -> new ScoreReef(io, pos.getAsInt() ,3)));
        controller.b().and(automated()).and(() -> io.shooter.hasCoral()).onTrue(Util.Do(() -> new ScoreReef(io, pos.getAsInt() ,2)));
        controller.a().and(automated()).and(() -> io.shooter.hasCoral()).onTrue(Util.Do(() -> new ScoreReef(io, pos.getAsInt() ,1)));
        controller.a().and(automated()).and(() -> !io.shooter.hasCoral()).onTrue(new Intake(io, false));

        controller.start().and(automated()).onTrue(Util.Do(() -> io.elevator.move(0)));
        controller.back().onTrue(Util.Do(io.chassis::resetAngle, io.chassis));
    }

    double direction = 1; 

    public void configureManual(){
        controller.back().and(manual()).onTrue(Util.Do(io.chassis::resetOdometry, io.chassis));

        // controller.y().and(manual()).onTrue(Util.Do(() -> io.elevator.move(4)));
        controller.x().and(manual()).onTrue(Util.Do(() -> io.elevator.move(3)));
        controller.b().and(manual()).onTrue(Util.Do(() -> io.elevator.move(2)));
        controller.a().and(manual()).onTrue(Util.Do(() -> io.elevator.move(1)));

        controller.leftBumper().onTrue(Util.Do(() -> io.shooter.speed(direction * .4), io.shooter)).onFalse(Util.Do(() -> io.shooter.volts(0), io.shooter));
        controller.rightBumper().onTrue(Util.Do(() -> io.shooter.speed(direction * 1), io.shooter)).onFalse(Util.Do(() -> io.shooter.volts(0), io.shooter));
        controller.y().and(manual()).onTrue(Util.Do(() -> { direction = -direction;}));
        
        // controller.povDown().and( manual() ).onTrue(Util.Do(io.chassis::toggle));
        // controller.povLeft().and( manual() ).onTrue(Util.Do(io.chassis::syncEncoders));
        // controller.povRight().and( manual() ).and(() -> {return !io.chassis.active;}).onTrue(new InstantCommand(io.chassis::zeroAbsolute)); // Add the Rumble effect
    }

    void configureCharacterisaton(){
 
        // controller.x().and(characterise()).toggleOnTrue(io.chassis.driveRoutine.quasistatic(Direction.kForward));
        // controller.a().and(characterise()).toggleOnTrue(io.chassis.driveRoutine.quasistatic(Direction.kReverse));
        // controller.y().and(characterise()).toggleOnTrue(io.chassis.driveRoutine.dynamic(Direction.kForward));
        // controller.b().and(characterise()).toggleOnTrue(io.chassis.driveRoutine.dynamic(Direction.kReverse));

        // controller.povUp().and(characterise()).toggleOnTrue(io.chassis.   steerRoutine.quasistatic(Direction.kForward));
        // controller.povDown().and(characterise()).toggleOnTrue(io.chassis. steerRoutine.quasistatic(Direction.kReverse));
        // controller.povRight().and(characterise()).toggleOnTrue(io.steerRoutine.dynamic(Direction.kForward));
        // controller.povLeft().and(characterise()).toggleOnTrue(io.chassis. steerRoutine.dynamic(Direction.kReverse));

        controller.x().and(characterise()).toggleOnTrue(io.shooter.pivotRoutine.quasistatic(Direction.kForward));
        controller.a().and(characterise()).toggleOnTrue(io.shooter.pivotRoutine.quasistatic(Direction.kReverse));
        controller.y().and(characterise()).toggleOnTrue(io.shooter.pivotRoutine.dynamic(Direction.kForward));
        controller.b().and(characterise()).toggleOnTrue(io.shooter.pivotRoutine.dynamic(Direction.kReverse));

        controller.leftBumper().and(characterise()).toggleOnTrue(io.elevator.routine.quasistatic(Direction.kForward));
        controller.rightBumper().and(characterise()).toggleOnTrue(io.elevator.routine.quasistatic(Direction.kReverse));
        controller.leftTrigger().and(characterise()).toggleOnTrue(io.elevator.routine.dynamic(Direction.kForward));
        controller.rightTrigger().and(characterise()).toggleOnTrue(io.elevator.routine.dynamic(Direction.kReverse));

    }

    void configureDebug(){
        controller.back().and(debug()).onTrue(Util.Do(io.chassis::resetOdometry, io.chassis));

        double volts = 3;

        controller.leftBumper().and(debug()).onTrue(Util.Do(() -> {
            io.elevator.volts(-volts / 2);
            io.elevator.stopped = true;
        }, io.elevator)).onFalse(Util.Do(() -> io.elevator.volts(0), io.elevator));
        controller.rightBumper().and(debug()).onTrue(Util.Do(() -> {
            io.elevator.volts(volts);
            io.elevator.stopped = true;
        }, io.elevator)).onFalse(Util.Do(() -> io.elevator.volts(0), io.elevator));

        controller.y().and(debug()).onTrue(Util.Do(() -> {
            io.shooter.volts(volts);
        }, io.shooter)).onFalse(Util.Do(() -> io.shooter.volts(0), io.shooter));


        controller.x().and(debug()).onTrue(Util.Do(io.elevator::zero, io.elevator));
        controller.a().and(debug()).onTrue(Util.Do(io.elevator::toggleSoftLimits, io.elevator));
         controller.povLeft().and(debug()).onTrue(Util.Do( () -> io.elevator.move(3),io.elevator));
        controller.povRight().and(debug()).onTrue(Util.Do( () -> io.elevator.move(2),io.elevator));
        controller.povDown().and(debug()).onTrue(Util.Do( () -> io.elevator.move(1),io.elevator));
    }

}
