// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants.BigStickConstants;

public class BigStick extends SubsystemBase {
  private CANSparkMax pivot;
  private RelativeEncoder bigStickEncoder;
  private SparkMaxPIDController bigStickController;

  private boolean deployed = false;

  private static final BigStick bigStick = new BigStick();

  public static BigStick getInstance(){
    return bigStick;
  }

  /** Creates a new BigStick. */
  public BigStick() {
    pivot = new PearadoxSparkMax(BigStickConstants.BIG_STICK_ID, MotorType.kBrushless, IdleMode.kBrake, 25, false,
      BigStickConstants.PIVOT_kP, BigStickConstants.PIVOT_kI, BigStickConstants.PIVOT_kD, 
      BigStickConstants.PIVOT_MIN_OUTPUT, BigStickConstants.PIVOT_MAX_OUTPUT);

    bigStickEncoder =  pivot.getEncoder();
    bigStickController = pivot.getPIDController();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Big Stick Position", bigStickEncoder.getPosition());

    Logger.getInstance().recordOutput("Big Stick/Position", bigStickEncoder.getPosition());
  }

  public void bigStickHold(){
    if(deployed){
      bigStickController.setReference(BigStickConstants.DEPLOYED_ROT, CANSparkMax.ControlType.kPosition, 0);
    }
    else{
      bigStickController.setReference(0, CANSparkMax.ControlType.kPosition, 0);
    }
  }

  public void toggleDeploy(){
    deployed = deployed ? false : true;
  }

  public boolean isDeployed(){
    return deployed;
  }
}
