// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  //POSITIVE IN/DEPLOY, NEGATIVE OUT/STOWED
  private PearadoxSparkMax driver;
  private PearadoxSparkMax pivot;

  private RelativeEncoder pivotEncoder;
  private SparkMaxPIDController intakeController;

  private boolean deployed = false;
  private boolean zeroed = false;

  private static final Intake intake = new Intake();

  public static Intake getInstance(){
    return intake;
  }

  /** Creates a new Intake. */
  public Intake() {
    driver = new PearadoxSparkMax(21, MotorType.kBrushless, IdleMode.kBrake, 40, true);
    pivot = new PearadoxSparkMax(22, MotorType.kBrushless, IdleMode.kBrake, 40, true);

    pivotEncoder = pivot.getEncoder();
    intakeController = pivot.getPIDController();
  }

  public void intakeIn(){
    driver.set(0.4);
  }

  public void intakeHold(){
    if(deployed){
      intakeController.setReference(IntakeConstants.DEPLOYED_ROT, CANSparkMax.ControlType.kPosition, 0);
      zeroed = false;
    }
    else{
      if(!zeroed && pivot.getOutputCurrent() > 38){
        zeroed = true;
        resetPivotEncoder();
      }
      if(!zeroed){
        pivot.set(-0.1);
      }
      if(zeroed){
        pivot.set(0);
      }
    }
  }

  public void intakeToggle(){
    if(!deployed){
      deployed = true;
    }
    else{
      deployed = false;
    }
  }

  public void intakeStop(){
    pivot.set(0);
  }

  public void resetPivotEncoder(){
    pivotEncoder.setPosition(0);
  }

  public void configPivotController(){
    intakeController.setP(IntakeConstants.PIVOT_kP, 0);
    intakeController.setI(IntakeConstants.PIVOT_kI, 0);
    intakeController.setD(IntakeConstants.PIVOT_kD, 0);
    intakeController.setFF(IntakeConstants.PIVOT_kFF, 0);
    intakeController.setOutputRange(IntakeConstants.PIVOT_MIN_OUTPUT, IntakeConstants.PIVOT_MAX_OUTPUT, 0);
    pivot.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pivot Position", pivotEncoder.getPosition());
    SmartDashboard.putBoolean("Deployed", deployed);
    SmartDashboard.putBoolean("Zeroed", zeroed);
    SmartDashboard.putNumber("Pivot Current", pivot.getOutputCurrent());
  }
}
