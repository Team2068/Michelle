package frc.robot.utility;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class Rumble {

    public Timer timer = new Timer();

    double duration;
    GenericHID controller;

    public Rumble(GenericHID controller){
        this.controller = controller;

    }

    Runnable[] patterns = {
        () -> { // Basic Shaking
            controller.setRumble(RumbleType.kBothRumble, .25);
        }
    };

    public void Run(double duration, int pattern){
        this.duration = duration;
        timer.start();
        patterns[pattern].run();
    }

    public boolean finished(){
        return timer.hasElapsed(duration);
    }

    public void End(){
        controller.setRumble(RumbleType.kBothRumble,0);
    }
}
