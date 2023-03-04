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

  private ArmMode mode = ArmMode.kZero;

  private static final Arm arm = new Arm();

  public static Arm getInstance(){
    return arm;
  }

  /** Creates a new Arm. */
  public Arm() {
    driver = new PearadoxSparkMax(ArmConstants.ARM_DRIVER_ID, MotorType.kBrushless, IdleMode.kBrake, 25, true);
    pivot = new PearadoxSparkMax(ArmConstants.ARM_PIVOT_ID, MotorType.kBrushless, IdleMode.kBrake, 40, true,
      ArmConstants.PIVOT_kP, ArmConstants.PIVOT_kI, ArmConstants.PIVOT_kD, 
      ArmConstants.PIVOT_MIN_OUTPUT, ArmConstants.PIVOT_MAX_OUTPUT);

    armEncoder = pivot.getEncoder();
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
      armController.setReference(ArmConstants.SUBS_UP_MODE_ROT, ControlType.kPosition);
    }
    else if(mode == ArmMode.kZero){
      armController.setReference(0.0, ControlType.kPosition);
    }
  }

  public void setZeroMode(){
    mode = ArmMode.kZero;
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

  public void intakeIn(double speed){
    driver.set(speed);
  }

  public void intakeOut(){
    driver.set(-0.9);
  }

  public void intakeStop(){
    driver.set(0);
  }

  public void armUp(){
    if(mode == ArmMode.kZero){
      mode = ArmMode.kLow;
    }
    else if(mode == ArmMode.kLow){
      mode = ArmMode.kMid;
    }
    else if(mode == ArmMode.kMid){
      mode = ArmMode.kHigh;
    }
    else if(mode == ArmMode.kSubs){
      mode = ArmMode.kHigh;
    }
  }

  public void armDown(){
    if(mode == ArmMode.kHigh){
      mode = ArmMode.kMid;
    }
    else if(mode == ArmMode.kSubs){
      mode = ArmMode.kMid;
    }
    else if(mode == ArmMode.kMid){
      mode = ArmMode.kLow;
    }
    else if(mode == ArmMode.kLow){
      mode = ArmMode.kZero;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Encoder", armEncoder.getPosition());
    if(mode == ArmMode.kZero){
      SmartDashboard.putString("Arm Mode", "kZero");
    }
    else if(mode == ArmMode.kLow){
      SmartDashboard.putString("Arm Mode", "kLow");
    }
    else if(mode == ArmMode.kMid){
      SmartDashboard.putString("Arm Mode", "kMid");
    }
    else if(mode == ArmMode.kHigh){
      SmartDashboard.putString("Arm Mode", "kHigh");
    }
    else if(mode == ArmMode.kSubs){
      SmartDashboard.putString("Arm Mode", "kSubs");
    }
  }
}
