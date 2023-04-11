// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants.IntakeConstants;

public class IntakeRollers extends SubsystemBase {
  private PearadoxSparkMax driver;

  private static final IntakeRollers intakeRollers = new IntakeRollers();

  public static IntakeRollers getInstance(){
    return intakeRollers;
  }

  /** Creates a new IntakeRollers. */
  public IntakeRollers() {
    driver = new PearadoxSparkMax(IntakeConstants.INTAKE_DRIVER_ID, MotorType.kBrushless, IdleMode.kBrake, 40, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intakeIn(double speed){
    driver.set(speed);
  }

  public void intakeOut(){
    driver.set(-0.4);
  }

  public void intakeStop(){
    driver.set(0);
  }
}
