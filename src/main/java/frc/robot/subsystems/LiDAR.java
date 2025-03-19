// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LiDAR extends SubsystemBase {
  public enum LidarConfiguration {
    DEFAULT, // Default mode, balanced performance
    SHORT_RANGE, // Short range, high speed
    DEFAULT_HIGH_SPEED, // Default range, higher speed short range
    MAXIMUM_RANGE, // Maximum range
    HIGH_SENSITIVE, // High sensitivity detection, high erroneous measurements
    LOW_SENSITIVE // Low sensitivity detection, low erroneous measurements
  }

  I2C _lidar;
  int config = 0;
  String[] display = {"Default", "Short Range", "Default High Speed", "Maximum Range", "High Sensetivity", "Low Sensetivity"};
  byte[] distanceBuffer = new byte[2];

  public LiDAR() {
    this(LidarConfiguration.DEFAULT, (byte) 0x62);
  }

  public LiDAR(LidarConfiguration configuration, int address) {
    _lidar = new I2C(Port.kMXP, 0x62); // default i2c port
    // if the address isn't 0x62 (default address) change the address
    if (address != 0x62) {
      setI2cAddress((byte) address);
    }

    changeMode(configuration);
  }

  private void setI2cAddress(byte newAddress) {
    // read and write back serial number
    // to ensure that we have the correct sensor
    byte[] dataBytes = new byte[2];
    _lidar.read(0x8F, 2, dataBytes);
    _lidar.write(0x18, dataBytes[0]);
    _lidar.write(0x19, dataBytes[1]);

    // Write the new I2C device address to registers
    dataBytes[0] = newAddress;
    _lidar.write(0x1a, dataBytes[0]);

    // Enable the new I2C device address using the default I2C device address
    dataBytes[0] = 0;
    _lidar.write(0x1e, dataBytes[0]);

    // delete old object, create new one at new address
    _lidar = null;
    _lidar = new I2C(Port.kMXP, newAddress);

    // disable default I2C device address (using the new I2C device address)
    dataBytes[0] = (1 << 3); // set bit to disable default address
    _lidar.write(0x1e, dataBytes[0]);
  }

  public void changeMode(LidarConfiguration configuration) {
    switch (configuration) {
      case DEFAULT: // Default mode, balanced performance
        _lidar.write(0x02, 0x80); // Default
        _lidar.write(0x04, 0x08); // Default
        _lidar.write(0x1c, 0x00); // Default
        break;

      case SHORT_RANGE: // Short range, high speed
        _lidar.write(0x02, 0x1d);
        _lidar.write(0x04, 0x08); // Default
        _lidar.write(0x1c, 0x00); // Default
        break;

      case DEFAULT_HIGH_SPEED: // Default range, higher speed short range
        _lidar.write(0x02, 0x80); // Default
        _lidar.write(0x04, 0x00);
        _lidar.write(0x1c, 0x00); // Default
        break;

      case MAXIMUM_RANGE: // Maximum range
        _lidar.write(0x02, 0xff);
        _lidar.write(0x04, 0x08); // Default
        _lidar.write(0x1c, 0x00); // Default
        break;

      case HIGH_SENSITIVE: // High sensitivity detection, high erroneous measurements
        _lidar.write(0x02, 0x80); // Default
        _lidar.write(0x04, 0x08); // Default
        _lidar.write(0x1c, 0x80);
        break;

      case LOW_SENSITIVE: // Low sensitivity detection, low erroneous measurements
        _lidar.write(0x02, 0x80); // Default
        _lidar.write(0x04, 0x08); // Default
        _lidar.write(0x1c, 0xb0);
        break;
    }
    config = configuration.ordinal();
  }

  public double distance(boolean biasCorrection) {
    _lidar.write(0x00, (biasCorrection) ? 0x04 : 0x03);

    _lidar.read(0x8F, 2, distanceBuffer);

    return ((distanceBuffer[0] << 8) + distanceBuffer[1]) * 10; // 
  }

  public void reset() {
    _lidar.write(0x00, 0x00); // Centimeters to meters
  }

  public LidarConfiguration getCurrentConfiguration() {
    return LidarConfiguration.values()[config];
  }

  @Override
  public void periodic() {
    double distance = 10 * distance(true);
    SmartDashboard.putNumber("Lidar Sensor Distance", distance);
    SmartDashboard.putString("Lidar Config", display[config]);
  }
}
