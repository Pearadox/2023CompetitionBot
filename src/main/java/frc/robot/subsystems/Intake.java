// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  //POSITIVE IN/DEPLOY, NEGATIVE OUT/STOWED
  private PearadoxSparkMax driver;
  private PearadoxSparkMax pivot;

  private double lastToggledTime;
  private boolean deployed = false;

  private static final Intake intake = new Intake();

  public static Intake getInstance(){
    return intake;
  }

  /** Creates a new Intake. */
  public Intake() {
    driver = new PearadoxSparkMax(21, MotorType.kBrushless, IdleMode.kBrake, 40, false);
    pivot = new PearadoxSparkMax(22, MotorType.kBrushless, IdleMode.kBrake, 40, false);

    lastToggledTime = Timer.getFPGATimestamp();
  }

  public void intakeIn(){
    driver.set(0.25);
  }

  public void intakeHold(){
    if(deployed){
      intakeIn();
      if(Timer.getFPGATimestamp() - lastToggledTime < IntakeConstants.INTAKE_DEPLOY_TIME){
        pivot.set(0.8);
      }
      else{
        pivot.set(0);
      }
    }
    else{
      if(Timer.getFPGATimestamp() - lastToggledTime < IntakeConstants.INTAKE_STOW_TIME){
        pivot.set(-0.8);
      }
      else{
        pivot.set(0);
      }
    }
  }

  public void intakeToggle(){
    if(!deployed){
      deployed = true;
      lastToggledTime = Timer.getFPGATimestamp();
    }
    else{
      deployed = false;
      lastToggledTime = Timer.getFPGATimestamp();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
