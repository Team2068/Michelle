package frc.robot.utility;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class AutomatedController {
    public final CommandXboxController controller;
    public final SendableChooser<Runnable> selector = new SendableChooser<Runnable>();
    public boolean manual = true;

    IO io;

    public AutomatedController(int port, IO io){
        this.io = io;

        selector.setDefaultOption("Automated", () -> {manual = false;});
        selector.addOption("Manual", () -> {manual = true;});

        selector.onChange((x) -> {x.run();}); // TODO: See if this allows us to have the selector always be up-to-date and add the same for auton testing

        controller = new CommandXboxController(port);
        controller.rightStick().onTrue(new InstantCommand(() -> io.chassis.field_oritented = !io.chassis.field_oritented));
        controller.leftStick().onTrue(new InstantCommand(io.chassis::resetOdometry));
        // controller.back().onTrue(Util.Do(io.elevator::rest));
        // controller.start().onTrue(Util.Do(io.elevator::zero));
        controller.start().onTrue(Util.Do( () -> manual = !manual)).debounce(3);
        configure();
    }
    public BooleanSupplier automated(){
        return () -> { return !manual; };
    }

    public BooleanSupplier manual(){
        return () -> { return manual; };
    }

    public BooleanSupplier toggleMode(){
        return () -> {
            manual = !manual;
            return manual;
        };
    }

    public void configure(){
        
        controller.start().and(controller.getHID()::getBackButtonPressed).onTrue(Util.Do(this::toggleMode));

        // AUTOMATED

        // Based on the nearest element and our field orientation
        // LB align Left and Score Coral & Score Barge
        // RB align Right and Score Coral & Score Processor 

        // controller.y().and( automated() ).onTrue(Util.Do(io.elevator::L4));
        // controller.b().and( automated() ).onTrue(Util.Do(io.elevator::L3));
        // controller.a().and( automated() ).onTrue(Util.Do(io.elevator::L2));
        // controller.x().and( automated() )
        //     .onTrue(Util.Do(
        //         new ReleaseAlgae(io, false),
        //         new GrabAlgae(io),
        //         io.intake::grabbed));

        // controller.povUp().and( automated() )
        //     .onTrue(Util.Do(() -> io.elevator.speed(0.25)))
        //     .onFalse(Util.Do(() -> io.elevator.speed(0.0)));

        // controller.povDown().and( automated() )
        //     .onTrue(Util.Do(() -> io.elevator.speed(-0.25)))
        //     .onFalse(Util.Do(() -> io.elevator.speed(0.0)));

        // controller.povLeft().and( automated() )
        //     .onTrue(Util.Do(() -> io.intake.speed(-0.5)))
        //     .onFalse(Util.Do(() -> io.intake.speed(0.0)));

        // controller.povRight().and( automated() )
        //     .onTrue(Util.Do(() -> io.intake.speed(0.5)))
        //     .onFalse(Util.Do(() -> io.intake.speed(0.0)));

        //     // MANUAL

        //     controller.rightTrigger().and( manual() )
        //     .onTrue(Util.Do(() -> io.elevator.speed(0.25)))
        //     .onFalse(Util.Do(() -> io.elevator.speed(0.0)));

        // controller.leftTrigger().and( manual() )
        //     .onTrue(Util.Do(() -> io.elevator.speed(-0.25)))
        //     .onFalse(Util.Do(() -> io.elevator.speed(0.0)));

        // controller.leftBumper().and( manual() )
        //     .onTrue(Util.Do(() -> io.intake.speed(-0.5)))
        //     .onFalse(Util.Do(() -> io.intake.speed(0.0)));

        // controller.rightBumper().and( manual() )
        //     .onTrue(Util.Do(() -> io.intake.speed(0.5)))
        //     .onFalse(Util.Do(() -> io.intake.speed(0.0)));

        controller.povUp().and( manual() ).onTrue(Util.Do(io.chassis::enable));
        controller.povDown().and( manual() ).onTrue(Util.Do(io.chassis::disable)).debounce(1.5);
        controller.povLeft().and( manual() ).onTrue(Util.Do(io.chassis::syncEncoders));
        controller.povRight().and( manual() ).and(() -> {return !io.chassis.active;}).onTrue(new Rumble(0, .5, controller.getHID(), io.chassis::resetAbsolute)); // Add the Rumble effect
        // controller.povRight().and( manual() ).and(() -> {return !io.chassis.active;}).onTrue(new InstantCommand(io.chassis::resetAbsolute)); // Add the Rumble effect
        }
}
