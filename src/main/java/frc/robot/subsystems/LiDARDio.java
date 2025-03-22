// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;

public class LiDARDio {

  Counter counter;

  public LiDARDio(int port) {
    counter = new Counter(new DigitalInput(port));
    counter.setMaxPeriod(1.0);
    counter.setSemiPeriodMode(true);   
    reset();
  }

  public double distance() {
    return (counter.getPeriod() * 100000000.0) - 18; // TODO: Check if this is in Metres
  }

  public boolean connected() {
    return (counter.get() != 0);
  }

  public void reset() {
    counter.reset();
  }
}
	