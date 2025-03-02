package frc.robot.utility;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.RotateChassis;
import frc.robot.commands.SimpleAlign;

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
        controller.back().onTrue(Util.Do(io.chassis::resetOdometry, io.chassis));

        // controller.leftBumper().onTrue(new SimpleAlign(io, false));
        // controller.rightBumper().onTrue(new SimpleAlign(io, true));
        
        controller.leftBumper().onTrue(new RotateChassis(io, 45));

        // AUTOMATED

        // Based on the nearest element and our field orientation
        // LB align Left and Score Coral & Score Barge
        // RB align Right and Score Coral & Score Processor 

        // controller.y().and( automated() ).onTrue(Util.D      
        controller.povDown().and( manual() ).onTrue(Util.Do(io.chassis::toggle));
        controller.povLeft().and( manual() ).onTrue(Util.Do(io.chassis::syncEncoders));
        controller.povRight().and( manual() ).and(() -> {return !io.chassis.active;}).onTrue(new InstantCommand(io.chassis::zeroAbsolute)); // Add the Rumble effect

        // controller.povRight().and( manual() ).and(() -> {return !io.chassis.active;}).onTrue(new Rumble(0, .5, controller.getHID(), io.chassis::zeroAbsolute)); // Add the Rumble effect
    }
}
