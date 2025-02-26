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
    public int mode = 0;

    final int AUTOMATED = 0;
    final int MANUAL = 1;
    final int DEBUG = 2;

    Rumble rumble;
    IO io;

    public AutomatedController(int port, IO io){
        this.io = io;

        selector.setDefaultOption("Automated", () -> {mode = 0;});
        selector.addOption("Manual", () -> {mode = 1;});
        selector.addOption("Debug", () -> {mode = 2;});

        selector.onChange((x) -> {x.run();});

        controller = new CommandXboxController(port);
        rumble = new Rumble(controller.getHID());

        controller.rightStick().onTrue(new InstantCommand(() -> io.chassis.field_oritented = !io.chassis.field_oritented));
        controller.leftStick().onTrue(new InstantCommand(io.chassis::resetOdometry));
        // controller.back().onTrue(Util.Do(io.elevator::rest));
        // controller.start().onTrue(Util.Do(io.elevator::zero));
        configure();
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

    public BooleanSupplier debug(){
        return mode(2);
    }

    public void switchMode(){
        mode = (mode + 1) % 3;
    }

    public void configure(){
        controller.start().and(controller.getHID()::getBackButtonPressed).onTrue(Util.Do(this::switchMode));
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

    private void configureDebug(){
        controller.a().and(debug()).toggleOnTrue(new Aimbot(io));
        controller.b().and(debug()).toggleOnTrue(new AutoAlign(0, io));
        controller.x().and(debug()).onTrue(Util.Do(io.chassis::resetAngle));
    }
}
