package frc.robot.utility;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Aimbot;
import frc.robot.commands.AutoAlign;

public class AutomatedController {
    public final CommandXboxController controller;
    public final SendableChooser<Runnable> selector = new SendableChooser<Runnable>();
    public boolean manual = true;

    Rumble rumble;
    IO io;

    public AutomatedController(int port, IO io){
        this.io = io;

        selector.setDefaultOption("Automated", () -> {manual = false;});
        selector.addOption("Manual", () -> {manual = true;});

        selector.onChange((x) -> {x.run();}); // TODO: See if this allows us to have the selector always be up-to-date and add the same for auton testing

        controller = new CommandXboxController(port);
        rumble = new Rumble(controller.getHID());

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

        // AUTOMATED

        // Based on the nearest element and our field orientation
        // LB align Left and Score Coral & Score Barge
        // RB align Right and Score Coral & Score Processor

        // controller.leftBumper().and(automated()).onTrue(new ConditionalCommand(new ScoreReef(io, false), new ScoreBarge(io),
        //     io.limelight::reefZone));

        controller.leftBumper().onTrue(new AutoAlign(0, io));
        controller.rightBumper().onTrue(new AutoAlign(2, io));

        // controller.rightBumper().and(automated()).onTrue(new ConditionalCommand(new ScoreReef(io, true), new ReleaseAlgae(io),
        //     io.limelight::reefZone));

        controller.a().toggleOnTrue(new Aimbot(io));
        controller.b().toggleOnTrue(new AutoAlign(0, io));

        controller.povDown().and( manual() ).onTrue(Util.Do(io.chassis::toggle));
        controller.povLeft().and( manual() ).onTrue(Util.Do(io.chassis::syncEncoders));
        controller.povRight().and( manual() ).and(() -> {return !io.chassis.active;}).onTrue(new InstantCommand(io.chassis::zeroAbsolute)); // Add the Rumble effect

        // controller.povRight().and(manual()).and(() -> {return !io.chassis.active;}).onTrue(Util.DoUntil(
        //         () -> {
        //             rumble.Run(.5, 0);
        //             io.chassis.zeroAbsolute();
        //         }, rumble::End, rumble::finished,
        // io.chassis));
        // controller.povRight().and( manual() ).and(() -> {return !io.chassis.active;}).onTrue(new Rumble(0, .5, controller.getHID(), io.chassis::zeroAbsolute)); // Add the Rumble effect
    }
}
