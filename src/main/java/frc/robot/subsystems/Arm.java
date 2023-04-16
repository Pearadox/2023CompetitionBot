// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.VisionConstants;

public class Arm extends SubsystemBase {
  private PearadoxSparkMax driver;
  private PearadoxSparkMax pivot;

  private RelativeEncoder driverEncoder;
  private RelativeEncoder pivotEncoder;

  private SparkMaxPIDController armController;

  private enum ArmMode{
    kHigh, kMid, kLow, kGroundCone, kSubs, kZero
  }

  private enum IntakeMode{
    kIn, kOut
  }

  private ArmMode armMode = ArmMode.kZero;
  private IntakeMode intakeMode = IntakeMode.kIn;
  private double armAdjust = 0;
  private boolean rumbled = false;

  private NetworkTable llTable = NetworkTableInstance.getDefault().getTable(VisionConstants.ARM_LL_NAME);
  private double[] cameraPose = new double[6];

  private static final Arm arm = new Arm();

  public static Arm getInstance(){
    return arm;
  }

  /** Creates a new Arm. */
  public Arm() {
    driver = new PearadoxSparkMax(ArmConstants.ARM_DRIVER_ID, MotorType.kBrushless, IdleMode.kBrake, 35, true);
    pivot = new PearadoxSparkMax(ArmConstants.ARM_PIVOT_ID, MotorType.kBrushless, IdleMode.kBrake, 40, true,
      ArmConstants.PIVOT_kP, ArmConstants.PIVOT_kI, ArmConstants.PIVOT_kD, 
      ArmConstants.PIVOT_MIN_OUTPUT, ArmConstants.PIVOT_MAX_OUTPUT);

    driverEncoder = driver.getEncoder();
    pivotEncoder = pivot.getEncoder();
    armController = pivot.getPIDController();

    llTable.getEntry("pipeline").setNumber(1);
  }

  public void armHold(){
    if(armMode == ArmMode.kLow){
      armController.setReference(ArmConstants.LOW_MODE_ROT + armAdjust, ControlType.kPosition);
    }
    else if(armMode == ArmMode.kGroundCone){
      armController.setReference(ArmConstants.GROUND_CONE_MODE_ROT + armAdjust, ControlType.kPosition);
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

  public void setGroundConeMode(){
    armMode = ArmMode.kGroundCone;
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
        driver.set(0.01);
      }
      else if(armMode == ArmMode.kSubs || armMode == ArmMode.kGroundCone){
        driver.set(1.0);
      }
      else{
        driver.set(0.35);
      }
    }
    else{
      driver.set(-1.0);
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
    else if(armMode == ArmMode.kLow || armMode == ArmMode.kGroundCone){
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
    else if(armMode == ArmMode.kMid || armMode == ArmMode.kGroundCone){
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

  public boolean hasCone(){
    return armMode == ArmMode.kSubs && Math.abs(driverEncoder.getVelocity()) < 0.1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(hasLLTarget()){
      cameraPose = llTable.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
    }

    SmartDashboard.putNumber("Arm Tz", getTz());
    SmartDashboard.putNumber("Arm Adjust", armAdjust);
    SmartDashboard.putNumber("Arm Position", pivotEncoder.getPosition());
    SmartDashboard.putNumber("Arm Velocity", driverEncoder.getVelocity());
    SmartDashboard.putString("Arm Mode", armMode.toString());
    SmartDashboard.putBoolean("Teleop Enabled", DriverStation.isTeleopEnabled());

    Logger.getInstance().recordOutput("Arm/Tz", getTz());
    Logger.getInstance().recordOutput("Arm/Adjust", armAdjust);
    Logger.getInstance().recordOutput("Arm/Pivot Position", pivotEncoder.getPosition());
    Logger.getInstance().recordOutput("Arm/Driver Velocity", driverEncoder.getVelocity());
    Logger.getInstance().recordOutput("Arm/Mode", armMode.toString());

    if(!rumbled && hasCone()){
      CommandScheduler.getInstance().schedule(rumbleController());
      rumbled = true;
    }
    if(rumbled && !hasCone()){
      rumbled = false;
    }

    if(DriverStation.isTeleopEnabled()){
      if(RobotContainer.opController.getRightTriggerAxis() > 0.85){
        if(RobotContainer.intake.isDeployed()){
          RobotContainer.intake.intakeToggle();
        }
        if(RobotContainer.bigStick.isDeployed()){
          RobotContainer.bigStick.toggleDeploy();
        }
        armMode = ArmMode.kGroundCone;
      }
    }
  }

  public boolean hasLLTarget(){
    return llTable.getEntry("tv").getDouble(0) != 0;
  }

  public double getTz(){
    return cameraPose[2];
  }

  public Command rumbleController(){
    return new InstantCommand(() -> RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 1))
      .andThen(new WaitCommand(0.75))
      .andThen(new InstantCommand(() -> RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 0)));
  }
}
