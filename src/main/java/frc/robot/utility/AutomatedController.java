package frc.robot.utility;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Aimbot;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.ClearAlgae;
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
        selector.addOption("Debug", () -> {mode = 2;});
        selector.addOption("Debug Setting", () -> {mode = 3;});

        selector.onChange((x) -> {x.run();});

        controller = new CommandXboxController(port);
        // rumble = new Rumble(controller.getHID());

        controller.rightStick().onTrue(new InstantCommand(() -> io.chassis.field_oritented = !io.chassis.field_oritented));
        controller.leftStick().debounce(2).onTrue(new InstantCommand(io.chassis::resetAngle));
        // controller.back().onTrue(Util.Do(io.elevator::rest));
        // controller.start().onTrue(Util.Do(io.elevator::zero));
        configure();
        configureAutomated();
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

    public BooleanSupplier debug_setting(){
        return mode(3);
    }

    public void switchMode(){
        mode = (mode + 1) % 3;
    }

    public void configure(){
        
        controller.start().and(controller.getHID()::getBackButtonPressed).onTrue(Util.Do(this::switchMode));
        controller.back().onTrue(Util.Do(io.chassis::resetOdometry, io.chassis));
  
        controller.povDown().and( manual() ).onTrue(Util.Do(io.chassis::toggle));
        controller.povLeft().and( manual() ).onTrue(Util.Do(io.chassis::syncEncoders));
        controller.povRight().and( manual() ).and(() -> {return !io.chassis.active;}).onTrue(new InstantCommand(io.chassis::zeroAbsolute)); // Add the Rumble effect
    }

    void configureAutomated(){
        controller.leftBumper().and(automated()).onTrue(Util.Do(() -> new LimelightAlign(io, 1, false)));
        controller.rightBumper().and(automated()).onTrue(Util.Do(() -> new LimelightAlign(io, 2, false)));
        
        controller.y().and(automated()).onTrue(new ScoreReef(io, 3));
        controller.b().and(automated()).onTrue(new ClearAlgae(io));
        controller.x().and(automated()).onTrue(new ScoreReef(io, 2));
        controller.a().and(automated()).onTrue(new ScoreReef(io, 1));

        controller.start().and(automated()).onTrue(Util.Do(() -> io.elevator.move(0)));
        controller.back().and(automated()).onTrue(Util.Do(() -> io.chassis.resetOdometry()));
    }

    void configureDebug(){
        controller.rightTrigger().and(debug()).toggleOnTrue(new Aimbot(io));
        controller.leftBumper().and(debug()).toggleOnTrue(new AutoAlign(0, io));
        controller.rightBumper().and(debug()).toggleOnTrue(new AutoAlign(2, io));
 
        controller.povUp().and(debug()).toggleOnTrue(io.chassis.driveRoutine.quasistatic(Direction.kForward));
        controller.povDown().and(debug()).toggleOnTrue(io.chassis.driveRoutine.quasistatic(Direction.kReverse));
        controller.povRight().and(debug()).toggleOnTrue(io.chassis.driveRoutine.dynamic(Direction.kForward));
        controller.povLeft().and(debug()).toggleOnTrue(io.chassis.driveRoutine.dynamic(Direction.kReverse));

        controller.x().and(debug()).toggleOnTrue(io.elevator.routine.quasistatic(Direction.kForward));
        controller.a().and(debug()).toggleOnTrue(io.elevator.routine.quasistatic(Direction.kReverse));
        controller.y().and(debug()).toggleOnTrue(io.elevator.routine.dynamic(Direction.kForward));
        controller.b().and(debug()).toggleOnTrue(io.elevator.routine.dynamic(Direction.kReverse));

        controller.leftBumper().and(debug_setting()).onTrue(Util.Do(() -> io.elevator.volts(-3), io.elevator)).onFalse(Util.Do(() -> io.elevator.volts(0), io.elevator));
        controller.rightBumper().and(debug_setting()).onTrue(Util.Do(() -> io.elevator.volts(3), io.elevator)).onFalse(Util.Do(() -> io.elevator.volts(0), io.elevator));
        controller.x().and(debug_setting()).onTrue(Util.Do(io.elevator::zero, io.elevator));
        // controller.x().and(debug_setting()).toggleOnTrue(new LimelightAlign(io, 1, false));

        controller.povUp().and(debug()).toggleOnTrue(io.chassis.   steerRoutine.quasistatic(Direction.kForward));
        controller.povDown().and(debug()).toggleOnTrue(io.chassis. steerRoutine.quasistatic(Direction.kReverse));
        controller.povRight().and(debug()).toggleOnTrue(io.chassis.steerRoutine.dynamic(Direction.kForward));
        controller.povLeft().and(debug()).toggleOnTrue(io.chassis. steerRoutine.dynamic(Direction.kReverse));

        controller.y().and(debug_setting()).onTrue(Util.Do( () -> io.elevator.move(4),io.elevator));
        controller.b().and(debug_setting()).onTrue(Util.Do( () -> io.elevator.move(3),io.elevator));
        controller.x().and(debug_setting()).onTrue(Util.Do( () -> io.elevator.move(2),io.elevator));
        controller.a().and(debug_setting()).onTrue(Util.Do( () -> io.elevator.move(1),io.elevator));
        controller.povUp().and(debug_setting()).onTrue(Util.Do(() -> io.elevator.move(5),io.elevator));
        
        controller.povLeft().and(debug_setting()).onTrue(Util.Do(() -> io.elevator.volts(4), io.elevator));
        controller.povDown().and(debug_setting()).onTrue(Util.Do(() -> io.elevator.volts(-4), io.elevator));
    }

}
