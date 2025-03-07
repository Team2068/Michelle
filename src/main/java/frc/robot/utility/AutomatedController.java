package frc.robot.utility;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ClearAlgae;
import frc.robot.commands.LimelightAlign;
import frc.robot.commands.ScoreReef;

public class AutomatedController {
    public final CommandXboxController controller;
    public final SendableChooser<Runnable> selector = new SendableChooser<Runnable>();
    public int mode = 0;
    
    public GenericHID rumble = new GenericHID(0);

    IO io;

    public AutomatedController(int port, IO io){
        this.io = io;

        selector.setDefaultOption("Automated", () -> {mode = 0;});
        selector.addOption("Manual", () -> {mode = 1;});
        selector.addOption("Tuning", () -> {mode = 2;});
        selector.addOption("Debug", () -> {mode = 3;});

        selector.onChange((x) -> {x.run();});

        controller = new CommandXboxController(port);
        // rumble = new Rumble(controller.getHID());

        controller.rightStick().onTrue(new InstantCommand(() -> io.chassis.field_oritented = !io.chassis.field_oritented));

        controller.start().and(controller.getHID()::getBackButtonPressed).onTrue(Util.Do(this::switchMode));

        configureManual();
        configureAutomated();
        configureTuning();
        configureDebug();
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

    public BooleanSupplier tuning(){
        return mode(2);
    }

    public BooleanSupplier debug(){
        return mode(3);
    }

    public void switchMode(){
        mode = (mode + 1) % 2;
        if (mode == 1)
            io.elevator.disableLimits();
        else
            io.elevator.enableLimits();

    }

    public void configureManual(){
        controller.back().and(manual()).onTrue(Util.Do(io.chassis::resetOdometry, io.chassis));

        controller.leftBumper().and( manual() ).onTrue(Util.Do(() -> io.elevator.volts(-3)));
        controller.rightBumper().and( manual() ).onTrue(Util.Do(() -> io.elevator.volts(3)));

        controller.x().and( manual() ).onTrue(Util.Do(io.claw::toggleStopper));
        controller.a().and( manual() ).onTrue(Util.Do(io.claw::toggleSlapper));
    }

    void configureAutomated(){
        controller.back().and(automated()).onTrue(Util.Do(io.chassis::resetAngle, io.chassis));

        controller.leftBumper().and(automated()).onTrue(Util.Do(() -> new LimelightAlign(io, 1, false)));
        controller.rightBumper().and(automated()).onTrue(Util.Do(() -> new LimelightAlign(io, 2, false)));
        
        controller.y().and(automated()).onTrue(new ScoreReef(io, 3));
        controller.b().and(automated()).onTrue(new ClearAlgae(io)); 
        controller.x().and(automated()).onTrue(new ScoreReef(io, 2));
        controller.a().and(automated()).onTrue(new ScoreReef(io, 1));

        // controller.start().and(automated()).onTrue(Util.Do(() -> io.elevator.move(0)));
        controller.back().and(automated()).onTrue(Util.Do(() -> io.chassis.resetOdometry()));
    }

    void configureTuning(){
        controller.x().and(tuning()).toggleOnTrue(io.chassis.driveRoutine.quasistatic(Direction.kForward));
        controller.a().and(tuning()).toggleOnTrue(io.chassis.driveRoutine.quasistatic(Direction.kReverse));
        controller.y().and(tuning()).toggleOnTrue(io.chassis.driveRoutine.dynamic(Direction.kForward));
        controller.b().and(tuning()).toggleOnTrue(io.chassis.driveRoutine.dynamic(Direction.kReverse));

        // controller.x().and(tuning()).toggleOnTrue(io.elevator.routine.quasistatic(Direction.kForward));
        // controller.a().and(tuning()).toggleOnTrue(io.elevator.routine.quasistatic(Direction.kReverse));
        // controller.y().and(tuning()).toggleOnTrue(io.elevator.routine.dynamic(Direction.kForward));
        // controller.b().and(tuning()).toggleOnTrue(io.elevator.routine.dynamic(Direction.kReverse));

        controller.povUp().and(tuning()).toggleOnTrue(io.chassis.   steerRoutine.quasistatic(Direction.kForward));
        controller.povDown().and(tuning()).toggleOnTrue(io.chassis. steerRoutine.quasistatic(Direction.kReverse));
        controller.povRight().and(tuning()).toggleOnTrue(io.chassis.steerRoutine.dynamic(Direction.kForward));
        controller.povLeft().and(tuning()).toggleOnTrue(io.chassis. steerRoutine.dynamic(Direction.kReverse));
    }

    void configureDebug(){
        controller.leftBumper().onTrue(Util.Do(() -> io.elevator.volts(-3), io.elevator)).onFalse(Util.Do(() -> io.elevator.volts(0), io.elevator));
        controller.rightBumper().onTrue(Util.Do(() -> io.elevator.volts(3), io.elevator)).onFalse(Util.Do(() -> io.elevator.volts(0), io.elevator));
        controller.start().and(debug()).onTrue(Util.Do(io.elevator::zero, io.elevator));


        controller.y().and(debug()).onTrue(Util.Do( () -> io.elevator.move(4),io.elevator));
        controller.b().and(debug()).onTrue(Util.Do( () -> io.elevator.move(3),io.elevator));
        controller.x().and(debug()).onTrue(Util.Do( () -> io.elevator.move(2),io.elevator));
        controller.a().and(debug()).onTrue(Util.Do( () -> io.elevator.move(1),io.elevator));
        controller.povUp().and(debug()).onTrue(Util.Do(() -> io.elevator.move(5),io.elevator));

        controller.povDown().and(debug()).onTrue(Util.Do(io.chassis::toggle));
        controller.povLeft().and(debug()).onTrue(Util.Do(io.chassis::syncEncoders));
        controller.povRight().and(debug()).and(() -> {return !io.chassis.active;}).onTrue(new InstantCommand(io.chassis::zeroAbsolute)); // Add the Rumble effect
        
        controller.povLeft().and(debug()).onTrue(Util.Do(() -> io.elevator.volts(4), io.elevator));
        controller.povDown().and(debug()).onTrue(Util.Do(() -> io.elevator.volts(-4), io.elevator));
    }

}
