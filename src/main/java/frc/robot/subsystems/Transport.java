// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;

public class Transport extends SubsystemBase {
  private PearadoxSparkMax feeder;

  private DigitalInput irSensor;
  private Debouncer debouncer = new Debouncer(0.2, DebounceType.kFalling);

  private boolean isHolding = true;
  private boolean rumbled = false;

  private static final Transport transport = new Transport();

  public static Transport getInstance(){
    return transport;
  }

  /** Creates a new Transport. */
  public Transport() {
    feeder = new PearadoxSparkMax(ShooterConstants.FEEDER_ID, MotorType.kBrushless, IdleMode.kBrake, 50, true);

    irSensor = new DigitalInput(1);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Ir Sensor", hasCube());
    Logger.getInstance().recordOutput("Transport/Distance Sensor", hasCube());

    if(isHolding){
      if(hasCube()){
        feederStop();
      }
      else{
        feederHold();
      }
    }

    if(!rumbled && hasCube()){
      CommandScheduler.getInstance().schedule(rumbleController());
      rumbled = true;
    }
    if(rumbled && !hasCube()){
      rumbled = false;
    }
  }

  public void feederHold(){
    feeder.set(0.4);
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

  public boolean hasCube(){
    return debouncer.calculate(!irSensor.get());
  }

  public void setHolding(boolean isHolding){
    this.isHolding = isHolding;
  }

  public Command rumbleController(){
    return new InstantCommand(() -> RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 1))
      .andThen(new WaitCommand(0.75))
      .andThen(new InstantCommand(() -> RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 0)));
  }
}
