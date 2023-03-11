// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;

public class Transport extends SubsystemBase {
  private PearadoxSparkMax feeder;

  private static final Transport transport = new Transport();
  private boolean hasCube = false;
  private boolean isDefault = true;

  public static Transport getInstance(){
    return transport;
  }

  /** Creates a new Transport. */
  public Transport() {
    feeder = new PearadoxSparkMax(ShooterConstants.FEEDER_ID, MotorType.kBrushless, IdleMode.kBrake, 50, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if(RobotContainer.shooter.hasCube() && !hasCube()){
    //   setHasCube(true);
    // }
    // if(isDefault){
    //   if(RobotContainer.transport.hasCube()){
    //     RobotContainer.transport.feederStop();
    //   }
    //   else{
    //     RobotContainer.transport.feederHold();
    //   }
    // }
  }

  public void feederHold(){
    feeder.set(0.1);
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

  public void setHasCube(boolean hasCube){
    this.hasCube = hasCube;
  }

  public boolean hasCube(){
    return hasCube;
  }

  public void setDefault(boolean b){
    isDefault = b;
  }
}
