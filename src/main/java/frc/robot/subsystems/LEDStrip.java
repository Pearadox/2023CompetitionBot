// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDStrip extends SubsystemBase {
  public static enum LED_MODE {
    Default, Solid
  }

  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;
  private final int numLEDs;
  private int[] currentColor = new int[]{0, 255, 0};

  private LED_MODE mode = LED_MODE.Default;
  // private int _index;

  public LEDStrip(int numberOfLeds, int port) {
    led = new AddressableLED(port);
    numLEDs = numberOfLeds;

    // Length is expensive to set, so only set it once, then just update data
    ledBuffer = new AddressableLEDBuffer(numLEDs);
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();

    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 0, 255, 0);
    }

    led.setData(ledBuffer);
  }

  public void setMode(LED_MODE mode) {
    this.mode = mode;
  }

  public void setColor(int r, int g, int b) {
    mode = LED_MODE.Solid;
    currentColor = new int[]{r, g, b};
  }

  // private void increment() {
  //   _index++;
  //   if (_index >= _ledBuffer.getLength()) {
  //     _index = 0;
  //   }
  // }

  private void ledHold() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 0, 255, 0);
    }
  }

  private void solid() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, currentColor[0], currentColor[1], currentColor[2]);
    }
  }

  @Override
  public void periodic() {
    switch (mode) {
      case Solid:
        solid();
        break;
      default:
        ledHold();
        break;
    }
    led.setData(ledBuffer);
  }
}