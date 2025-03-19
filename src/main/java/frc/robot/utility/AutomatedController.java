package frc.robot.utility;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.LimelightAlign;
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
        // controller.back().onTrue(Util.Do(io.elevator::rest));
        // controller.start().onTrue(Util.Do(io.elevator::zero));
        configureManual();
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
        controller.back().onTrue(Util.Do(io.chassis::resetOdometry, io.chassis));

        // controller.leftBumper().onTrue(new SimpleAlign(io, false));
        // controller.rightBumper().onTrue(new SimpleAlign(io, true));
        
        // controller.leftBumper().onTrue(new RotateChassis(io, 45));
        // controller.leftBumper().toggleOnTrue(new LimelightAlign(io, 0));
        // controller.rightBumper().toggleOnTrue(new LimelightAlign(io, 2));

        // AUTOMATED

        // Based on the nearest element and our field orientation
        // LB align Left and Score Coral & Score Barge
        // RB align Right and Score Coral & Score Processor

        controller.leftBumper().and(automated()).toggleOnTrue(new AutoAlign(0, io));
        controller.rightBumper().and(automated()).toggleOnTrue(new AutoAlign(2, io));

        // MANUAL
        // RB align Right and Score Coral & Score Processor 
        // controller.y().and( automated() ).onTrue(Util.D      
        controller.povDown().and( manual() ).onTrue(Util.Do(io.chassis::toggle));
        controller.povLeft().and( manual() ).onTrue(Util.Do(io.chassis::syncEncoders));
        controller.povRight().and( manual() ).and(() -> {return !io.chassis.active;}).onTrue(new InstantCommand(io.chassis::zeroAbsolute)); // Add the Rumble effect

        // controller.povRight().and( manual() ).and(() -> {return !io.chassis.active;}).onTrue(new Rumble(0, .5, controller.getHID(), io.chassis::zeroAbsolute)); // Add the Rumble effect
    }

    void configureAutomated(){
        controller.leftBumper().and(automated()).onTrue(Util.Do(() -> new LimelightAlign(io, 1, false)));
        controller.rightBumper().and(automated()).onTrue(Util.Do(() -> new LimelightAlign(io, 2, false)));

        // controller.a().and(automated()).and(() -> !io.claw.hasCoral()).onTrue(Util.Do(() -> new Intake(io, true, false, rumble)));

        controller.a().and(automated()).and(() -> io.claw.hasCoral()).onTrue(Util.Do(() -> new ScoreReef(io, 1, rumble)));
        controller.b().and(automated()).and(() -> io.claw.hasCoral()).onTrue(Util.Do(() -> new ScoreReef(io, 3, rumble)));
        controller.x().and(automated()).and(() -> io.claw.hasCoral()).onTrue(Util.Do(() -> new ScoreReef(io, 2, rumble)));
        controller.y().and(automated()).and(() -> io.claw.hasCoral()).onTrue(Util.Do(() -> new ScoreReef(io, 4, rumble)));

        // controller.povLeft().and(automated()).onTrue(Util.Do(() -> new Intake(io, false, true, rumble)));

        controller.start().and(automated()).onTrue(Util.Do(() -> io.elevator.move(0)));
        controller.back().and(automated()).onTrue(Util.Do(() -> io.chassis.resetOdometry()));
    }

    public void configureManual(){
        
        controller.start().and(controller.getHID()::getBackButtonPressed).onTrue(Util.Do(this::switchMode));
        controller.back().onTrue(Util.Do(io.chassis::resetOdometry, io.chassis));

        // controller.leftBumper().and(automated()).toggleOnTrue(new AutoAlign(0, io));
        // controller.rightBumper().and(automated()).toggleOnTrue(new AutoAlign(2, io));

        controller.povDown().and( manual() ).onTrue(Util.Do(io.chassis::toggle));
        controller.povLeft().and( manual() ).onTrue(Util.Do(io.chassis::syncEncoders));
        controller.povRight().and( manual() ).and(() -> {return !io.chassis.active;}).onTrue(new InstantCommand(io.chassis::zeroAbsolute)); // Add the Rumble effect

        // controller.povRight().and( manual() ).and(() -> {return !io.chassis.active;}).onTrue(new Rumble(0, .5, controller.getHID(), io.chassis::zeroAbsolute)); // Add the Rumble effect
    }

    void configureCharacterisaton(){
 
        controller.x().and(characterise()).toggleOnTrue(io.chassis.driveRoutine.quasistatic(Direction.kForward));
        controller.a().and(characterise()).toggleOnTrue(io.chassis.driveRoutine.quasistatic(Direction.kReverse));
        controller.y().and(characterise()).toggleOnTrue(io.chassis.driveRoutine.dynamic(Direction.kForward));
        controller.b().and(characterise()).toggleOnTrue(io.chassis.driveRoutine.dynamic(Direction.kReverse));

        controller.leftBumper().and(characterise()).toggleOnTrue(io.elevator.routine.quasistatic(Direction.kForward));
        controller.rightBumper().and(characterise()).toggleOnTrue(io.elevator.routine.quasistatic(Direction.kReverse));
        controller.leftTrigger().and(characterise()).toggleOnTrue(io.elevator.routine.dynamic(Direction.kForward));
        controller.rightTrigger().and(characterise()).toggleOnTrue(io.elevator.routine.dynamic(Direction.kReverse));

        // controller.povUp().and(characterise()).toggleOnTrue(io.chassis.   steerRoutine.quasistatic(Direction.kForward));
        // controller.povDown().and(characterise()).toggleOnTrue(io.chassis. steerRoutine.quasistatic(Direction.kReverse));
        // controller.povRight().and(characterise()).toggleOnTrue(io.chassis.steerRoutine.dynamic(Direction.kForward));
        // controller.povLeft().and(characterise()).toggleOnTrue(io.chassis. steerRoutine.dynamic(Direction.kReverse));
    }

    void configureDebug(){
        // controller.rightTrigger().and(debug()).toggleOnTrue(new Aimbot(io));
        // controller.leftBumper().and(debug()).toggleOnTrue(new AutoAlign(0, io));
        // controller.rightBumper().and(debug()).toggleOnTrue(new AutoAlign(2, io));

        // double volts = (double) Util.get("Test Elevator Volts", 1);
        double volts = 8;

        controller.leftBumper().and(debug()).onTrue(Util.Do(() -> {
            io.elevator.volts(-volts / 2);
            io.elevator.stopped = true;
        }, io.elevator)).onFalse(Util.Do(() -> io.elevator.volts(0), io.elevator));
        controller.rightBumper().and(debug()).onTrue(Util.Do(() -> {
            io.elevator.volts(volts);
            io.elevator.stopped = true;
        }, io.elevator)).onFalse(Util.Do(() -> io.elevator.volts(0), io.elevator));
        controller.x().and(debug()).onTrue(Util.Do(io.elevator::zero, io.elevator));
        controller.a().and(debug()).onTrue(Util.Do(io.elevator::toggleSoftLimits, io.elevator));
        
        // controller.x().and(debug_setting()).toggleOnTrue(new LimelightAlign(io, 1, false));

        controller.povUp().and(debug()).onTrue(Util.Do( () -> io.elevator.move(4),io.elevator));
        controller.povLeft().and(debug()).onTrue(Util.Do( () -> io.elevator.move(3),io.elevator));
        controller.povRight().and(debug()).onTrue(Util.Do( () -> io.elevator.move(2),io.elevator));
        controller.povDown().and(debug()).onTrue(Util.Do( () -> io.elevator.move(1),io.elevator));
        // controller.povUp().and(debug()).onTrue(Util.Do(() -> io.elevator.move(5),io.elevator));
        
        // controller.povLeft().and(debug_setting()).onTrue(Util.Do(() -> io.elevator.volts(4), io.elevator));
        // controller.povDown().and(debug_setting()).onTrue(Util.Do(() -> io.elevator.volts(-4), io.elevator));
    }

}
