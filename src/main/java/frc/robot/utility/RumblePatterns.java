package frc.robot.utility;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class RumblePatterns {

    public Timer timer = new Timer();

    public void Run(int pattern, GenericHID controller){
        switch(pattern){
            default:
                controller.setRumble(RumbleType.kBothRumble, .5);
            break;

            case 1:
                if (timer.get()/5.0 % 2.0 == 0.0)
                    controller.setRumble(RumbleType.kBothRumble, .5);
                else
                    controller.setRumble(RumbleType.kBothRumble, 0.0);
            break;
        }
    }
}
