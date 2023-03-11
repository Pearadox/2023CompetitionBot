// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private PearadoxSparkMax driver;
  private PearadoxSparkMax pivot;

  private RelativeEncoder armEncoder;

  private SparkMaxPIDController armController;

  private enum ArmMode{
    kHigh, kMid, kLow, kSubs, kZero
  }

  private enum IntakeMode{
    kIn, kOut
  }

  private ArmMode armMode = ArmMode.kZero;
  private IntakeMode intakeMode = IntakeMode.kIn;
  private double armAdjust = 0;

  private static final Arm arm = new Arm();

  public static Arm getInstance(){
    return arm;
  }

  /** Creates a new Arm. */
  public Arm() {
    driver = new PearadoxSparkMax(ArmConstants.ARM_DRIVER_ID, MotorType.kBrushless, IdleMode.kBrake, 15, true);
    pivot = new PearadoxSparkMax(ArmConstants.ARM_PIVOT_ID, MotorType.kBrushless, IdleMode.kBrake, 40, true,
      ArmConstants.PIVOT_kP, ArmConstants.PIVOT_kI, ArmConstants.PIVOT_kD, 
      ArmConstants.PIVOT_MIN_OUTPUT, ArmConstants.PIVOT_MAX_OUTPUT);

    armEncoder = pivot.getEncoder();
    armController = pivot.getPIDController();
  }

  public void armHold(){
    if(armMode == ArmMode.kLow){
      armController.setReference(ArmConstants.LOW_MODE_ROT + armAdjust, ControlType.kPosition);
    }
    else if(armMode == ArmMode.kMid){
      armController.setReference(ArmConstants.MID_MODE_ROT + armAdjust, ControlType.kPosition);
    }
    else if(armMode == ArmMode.kHigh){
      armController.setReference(ArmConstants.HIGH_MODE_ROT + armAdjust, ControlType.kPosition);
    }
    else if(armMode == ArmMode.kSubs){
      armController.setReference(ArmConstants.SUBS_MODE_ROT + armAdjust, ControlType.kPosition);
    }
    else if(armMode == ArmMode.kZero){
      armController.setReference(0.0, ControlType.kPosition);
    }
  }

  public void setZeroMode(){
    armMode = ArmMode.kZero;
  }

  public void setLowMode(){
    armMode = ArmMode.kLow;
  }

  public void setMidMode(){
    armMode = ArmMode.kMid;
  }

  public void setHighMode(){
    armMode = ArmMode.kHigh;
  }

  public void setSubsMode(){
    armMode = ArmMode.kSubs;
  }

  public boolean isDeployed(){
    return !(armMode == ArmMode.kZero);
  }

  public void intakeHold(){
    if(intakeMode == IntakeMode.kIn){
      if(armMode == ArmMode.kZero){
        intakeStop();
      }
      else if(armMode == ArmMode.kSubs){
        driver.set(0.65);
      }
      else{
        driver.set(0.15);
      }
    }
    else{
      driver.set(-0.9);
    }
  }

  public void intakeStop(){
    driver.set(0);
  }

  public void intakeIn(){
    intakeMode = IntakeMode.kIn;
  }

  public void intakeOut(){
    intakeMode = IntakeMode.kOut;
  }

  public void armUp(){
    if(armMode == ArmMode.kZero){
      armMode = ArmMode.kLow;
    }
    else if(armMode == ArmMode.kLow){
      armMode = ArmMode.kMid;
    }
    else if(armMode == ArmMode.kMid){
      armMode = ArmMode.kHigh;
    }
    else if(armMode == ArmMode.kSubs){
      armMode = ArmMode.kHigh;
    }
  }

  public void armDown(){
    if(armMode == ArmMode.kHigh){
      armMode = ArmMode.kMid;
    }
    else if(armMode == ArmMode.kSubs){
      armMode = ArmMode.kMid;
    }
    else if(armMode == ArmMode.kMid){
      armMode = ArmMode.kLow;
    }
    else if(armMode == ArmMode.kLow){
      armMode = ArmMode.kZero;
    }
  }

  public void armAdjustUp(){
    armAdjust += 0.2;
  }

  public void armAdjustDown(){
    armAdjust -= 0.2;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Adjust", armAdjust);

    SmartDashboard.putNumber("Arm Encoder", armEncoder.getPosition());
    if(armMode == ArmMode.kZero){
      SmartDashboard.putString("Arm Mode", "kZero");
    }
    else if(armMode == ArmMode.kLow){
      SmartDashboard.putString("Arm Mode", "kLow");
    }
    else if(armMode == ArmMode.kMid){
      SmartDashboard.putString("Arm Mode", "kMid");
    }
    else if(armMode == ArmMode.kHigh){
      SmartDashboard.putString("Arm Mode", "kHigh");
    }
    else if(armMode == ArmMode.kSubs){
      SmartDashboard.putString("Arm Mode", "kSubs");
    }
  }
}
