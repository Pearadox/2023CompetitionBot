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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  //POSITIVE IN/DEPLOY, NEGATIVE OUT/STOWED
  private PearadoxSparkMax pivot;

  private RelativeEncoder pivotEncoder;
  private SparkMaxPIDController intakeController;

  private double intakeAdjust = 0;
  private boolean deployed = false;
  private boolean zeroing = false;

  private static final Intake intake = new Intake();

  public static Intake getInstance(){
    return intake;
  }

  /** Creates a new Intake. */
  public Intake() {
    pivot = new PearadoxSparkMax(IntakeConstants.INTAKE_PIVOT_ID, MotorType.kBrushless, IdleMode.kBrake, 40, true,
      IntakeConstants.PIVOT_kP, IntakeConstants.PIVOT_kI, IntakeConstants.PIVOT_kD, 
      IntakeConstants.PIVOT_MIN_OUTPUT, IntakeConstants.PIVOT_MAX_OUTPUT);

    pivotEncoder = pivot.getEncoder();
    intakeController = pivot.getPIDController();
  }

  public void intakeHold(){
    if(zeroing){
      pivot.set(-0.25);
    }
    else if(deployed){
      intakeController.setReference(IntakeConstants.DEPLOYED_ROT + intakeAdjust, CANSparkMax.ControlType.kPosition, 0);
    }
    else{
      intakeController.setReference(0 + intakeAdjust, CANSparkMax.ControlType.kPosition, 0);
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

  public void intakeAdjustUp(){
    intakeAdjust += 0.5;
  }

  public void intakeAdjustDown(){
    intakeAdjust -= 0.5;
  }

  public void setZeroing(boolean zeroing){
    this.zeroing = zeroing;
  }

  public boolean isDeployed(){
    return deployed;
  }

  public void resetPivotEncoder(){
    pivotEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pivot Position", pivotEncoder.getPosition());
    SmartDashboard.putBoolean("Deployed", deployed);
    SmartDashboard.putNumber("Pivot Current", pivot.getOutputCurrent());

    Logger.getInstance().recordOutput("Intake/Pivot Position", pivotEncoder.getPosition());
    Logger.getInstance().recordOutput("Intake/Deployed", deployed);
    Logger.getInstance().recordOutput("Intake/Pivot Current", pivot.getOutputCurrent());

    if(DriverStation.isTeleopEnabled()){
      if((RobotContainer.driverController.getLeftTriggerAxis() > 0.5 || RobotContainer.opController.getLeftTriggerAxis() > 0.7)){
        if(RobotContainer.bigStick.isDeployed()){
          RobotContainer.bigStick.toggleDeploy();
        }
        if(RobotContainer.arm.isDeployed()){
          RobotContainer.arm.setZeroMode();
        }
        deployed = true;
      }
      else{
        deployed = false;
      }
    }
  }
}
