// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private PearadoxSparkMax driver;
  private PearadoxSparkMax pivot;

  private SparkMaxPIDController armController;

  private enum ArmMode{
    kHigh, kMid, kLow, kSubs
  }

  private ArmMode mode = ArmMode.kLow;

  private static final Arm arm = new Arm();

  public static Arm getInstance(){
    return arm;
  }

  /** Creates a new Arm. */
  public Arm() {
    driver = new PearadoxSparkMax(23, MotorType.kBrushless, IdleMode.kBrake, 40, false);
    pivot = new PearadoxSparkMax(24, MotorType.kBrushless, IdleMode.kBrake, 40, false);

    armController = pivot.getPIDController();
  }

  public void armHold(){
    if(mode == ArmMode.kLow){
      armController.setReference(ArmConstants.LOW_MODE_ROT, ControlType.kPosition);
    }
    else if(mode == ArmMode.kMid){
      armController.setReference(ArmConstants.MID_MODE_ROT, ControlType.kPosition);
    }
    else if(mode == ArmMode.kHigh){
      armController.setReference(ArmConstants.HIGH_MODE_ROT, ControlType.kPosition);
    }
    else if(mode == ArmMode.kSubs){
      armController.setReference(ArmConstants.SUBS_MODE_ROT, ControlType.kPosition);
    }
  }

  public void setLowMode(){
    mode = ArmMode.kLow;
  }

  public void setMidMode(){
    mode = ArmMode.kMid;
  }

  public void setHighMode(){
    mode = ArmMode.kHigh;
  }

  public void setSubsMode(){
    mode = ArmMode.kSubs;
  }

  public void intakeIn(){
    driver.set(0.5);
  }

  public void intakeOut(){
    driver.set(-1.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
