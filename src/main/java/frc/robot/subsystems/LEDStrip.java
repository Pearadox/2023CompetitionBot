// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class LEDStrip extends SubsystemBase {
  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;
  private final int numLEDs;

  private ArrayList<String> defaultColors = new ArrayList<>();

  private double lastShifted;

  private enum LEDMode{
    kDefault, kCone, kCube
  }
  private LEDMode mode = LEDMode.kDefault;

  public LEDStrip(int numberOfLeds, int port) {
    led = new AddressableLED(port);
    numLEDs = numberOfLeds;

    // Length is expensive to set, so only set it once, then just update data
    ledBuffer = new AddressableLEDBuffer(numLEDs);
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();

    lastShifted = Timer.getFPGATimestamp();
    loadDefaultColors();
  }

  public void loadDefaultColors(){
    for (int i = 0; i < ledBuffer.getLength(); i+=10) {
      if(i == 100){
        defaultColors.add("Y1");
        defaultColors.add("Y1");
        defaultColors.add("Y2");
        defaultColors.add("Y2");
        defaultColors.add("M");
        defaultColors.add("M");
        defaultColors.add("G2");
        defaultColors.add("G2");
        defaultColors.add("G1");
        defaultColors.add("G1");
      }
      else if(i % 20 == 0){
        if(i % 40 == 0){
          for(int n = 0; n < 10; n++){
            defaultColors.add("G");
          }
        }
        else{
          for(int n = 0; n < 10; n++){
            defaultColors.add("Y");
          }
        }
      }
      else{
        if((i + 10) % 40 == 0){
            defaultColors.add("Y1");
            defaultColors.add("Y1");
            defaultColors.add("Y2");
            defaultColors.add("Y2");
            defaultColors.add("M");
            defaultColors.add("M");
            defaultColors.add("G2");
            defaultColors.add("G2");
            defaultColors.add("G1");
            defaultColors.add("G1");
        }
        else{
              defaultColors.add("G1");
              defaultColors.add("G1");
            defaultColors.add("G2");
            defaultColors.add("G2");
            defaultColors.add("M");
            defaultColors.add("M");
            defaultColors.add("Y2");
            defaultColors.add("Y2");
            defaultColors.add("Y1");
            defaultColors.add("Y1");
        }
      // for(int n = 0; n < 5; n++){
      //   defaultColors.add("O");
      // }
      }
    }
  }

  public void shiftDefaultColors(){
    String temp = defaultColors.get(defaultColors.size() - 1);
    defaultColors.add(0, temp);
    defaultColors.remove(defaultColors.size() - 1);
  }

  public void animateDefault(){
    for(int i = 0; i < ledBuffer.getLength(); i++){
      switch(defaultColors.get(i)){
        case "G":
          ledBuffer.setRGB(i, 0, 255, 0);
          break;
        case "Y":
          ledBuffer.setRGB(i, 255, 120, 0);
          break;
        case "G1":
          ledBuffer.setRGB(i, 43, 232, 0);
          break;
        case "G2":
          ledBuffer.setRGB(i, 85, 210, 0);
          break;
        case "M":
          ledBuffer.setRGB(i, 128, 187, 0);
          break;
        case "Y2":
          ledBuffer.setRGB(i, 170, 165, 0);
          break;
        case "Y1":
          ledBuffer.setRGB(i, 213, 142, 0);
          break;
        // case "O":
        //   ledBuffer.setRGB(i, 0, 0, 0);
        //   break;
        default:
          ledBuffer.setRGB(i, 0, 0, 0);
      }
    }
  }

  // public void animateTest()
  // {
  //   int colorShifterCounter = 0;
  //   boolean ifGreen = true;
  //   for(int i = 0; i < ledBuffer.getLength(); i++)
  //   {
  //     if(defaultColors.get(i) == "G")
  //     {

  //     }
  //   }
  // }

  public ArrayList<String> getDefaultColors(){
    return defaultColors;
  }

  public void setDefaultMode(){
    mode = LEDMode.kDefault;
  }

  public void setConeMode(){
    mode = LEDMode.kCone;
  }

  public void setCubeMode(){
    mode = LEDMode.kCube;
  }

  public void setColor(int r, int g, int b) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, r, g, b);
    }
  }

  public void ledHold() {
    if(RobotContainer.transport.getIRSensor()){
      setColor(200, 0, 0);
    }
    else{
      if(mode == LEDMode.kCone){
        setColor(255, 120, 0);
      }
      else if(mode == LEDMode.kCube){
        setColor(100, 0, 127);
      }
      else{
        animateDefault();
      }
    }
  }

  @Override
  public void periodic() {
    if(Timer.getFPGATimestamp() - lastShifted > 0.1){
      shiftDefaultColors();
      lastShifted = Timer.getFPGATimestamp();
    }
    ledHold();
    led.setData(ledBuffer);
    SmartDashboard.putString("Default Colors 1st", defaultColors.get(0));
  }
}