package frc.robot.utility;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class RumblePatterns {
    public static void Run(int pattern, GenericHID controller){
        switch(pattern){
            default:
                controller.setRumble(RumbleType.kBothRumble, .5);
            break;
        }
    }
}
