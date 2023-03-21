// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants.ShooterConstants;

public class Transport extends SubsystemBase {
  private PearadoxSparkMax feeder;

  private DigitalInput distSensor;
  private DigitalInput irSensor;

  private boolean isHolding = true;

  private static final Transport transport = new Transport();

  public static Transport getInstance(){
    return transport;
  }

  /** Creates a new Transport. */
  public Transport() {
    feeder = new PearadoxSparkMax(ShooterConstants.FEEDER_ID, MotorType.kBrushless, IdleMode.kBrake, 50, true);

    distSensor = new DigitalInput(0);
    irSensor = new DigitalInput(1);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Dist Sensor", getDistSensor());
    SmartDashboard.putBoolean("Ir Sensor", getIRSensor());

    if(isHolding){
      if(getDistSensor() || getIRSensor()){
        feederStop();
      }
      else{
        feederHold();
      }
    }
  }

  public void feederHold(){
    feeder.set(0.15);
  }

  public void feederOut(double speed){
    feeder.set(speed);
  }

  public void feederStop(){
    feeder.set(0);
  }

  public void feederShoot(){
    feeder.set(0.7);
  }

  public boolean getDistSensor(){
    return !distSensor.get();
  }

  public boolean getIRSensor(){
    return !irSensor.get();
  }

  public void setHolding(boolean isHolding){
    this.isHolding = isHolding;
  }
}
