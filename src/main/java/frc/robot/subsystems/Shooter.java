// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.lib.util.LerpTable;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private PearadoxSparkMax topShooter;
  private PearadoxSparkMax botShooter;

  private RelativeEncoder topEncoder;
  private RelativeEncoder botEncoder;

  private SparkMaxPIDController topController;
  private SparkMaxPIDController botController;
  // private SimpleMotorFeedforward topShooterFeedforward = new SimpleMotorFeedforward(ShooterConstants.TOP_SHOOTER_kS, ShooterConstants.TOP_SHOOTER_kV, ShooterConstants.TOP_SHOOTER_kA);
  // private SimpleMotorFeedforward botShooterFeedforward = new SimpleMotorFeedforward(ShooterConstants.BOT_SHOOTER_kS, ShooterConstants.BOT_SHOOTER_kV, ShooterConstants.BOT_SHOOTER_kA);

  private DigitalInput irSensor;

  private NetworkTable llTable = NetworkTableInstance.getDefault().getTable("limelight-shooter");
  private LerpTable shooterLerp;
  private double target;
  // private double[] targetPose;

  // private double dist;
  // private MedianFilter distFilter = new MedianFilter(5);

  private enum ShooterMode{
    kHigh, kMid, kCS
  }
  private ShooterMode mode = ShooterMode.kHigh;

  private static final Shooter shooter = new Shooter();

  public static Shooter getInstance(){
    return shooter;
  }

  /** Creates a new Shooter. */
  public Shooter() {
    topShooter = new PearadoxSparkMax(ShooterConstants.TOP_SHOOTER_ID, MotorType.kBrushless, IdleMode.kBrake, 50, true,
      ShooterConstants.TOP_SHOOTER_kP, ShooterConstants.TOP_SHOOTER_kI, ShooterConstants.TOP_SHOOTER_kD,
      ShooterConstants.SHOOTER_MIN_OUTPUT, ShooterConstants.SHOOTER_MAX_OUTPUT);
    botShooter = new PearadoxSparkMax(ShooterConstants.BOT_SHOOTER_ID, MotorType.kBrushless, IdleMode.kBrake, 50, false,
      ShooterConstants.BOT_SHOOTER_kP, ShooterConstants.BOT_SHOOTER_kI, ShooterConstants.BOT_SHOOTER_kD,
      ShooterConstants.SHOOTER_MIN_OUTPUT, ShooterConstants.SHOOTER_MAX_OUTPUT);

    topEncoder = topShooter.getEncoder();
    botEncoder = botShooter.getEncoder();

    topController = topShooter.getPIDController();
    botController = botShooter.getPIDController();

    irSensor = new DigitalInput(0);

    shooterLerp = new LerpTable();

    //SHOOTER LOOKUP TABLE: (speed (rpm), distance (meters))
    shooterLerp.addPoint(0, 0);
  }

  public void shooterHold(){
    if(mode == ShooterMode.kCS){
      topController.setReference(
        target + 1.05,
        CANSparkMax.ControlType.kVoltage,
        0);
  
      botController.setReference(
        target - 1.25,
        CANSparkMax.ControlType.kVoltage,
        0);
    }
    else if(mode == ShooterMode.kHigh){
      topController.setReference(
        target-0.2,
        CANSparkMax.ControlType.kVoltage,
        0);
  
      botController.setReference(
        target+0.7,
        CANSparkMax.ControlType.kVoltage,
        0);
    }
    else{
      topController.setReference(
      target,
      CANSparkMax.ControlType.kVoltage,
      0);

    botController.setReference(
      target,
      CANSparkMax.ControlType.kVoltage,
      0);
    }
  }

  public boolean hasCube(){
    return !irSensor.get();
  }

  // public void toggleMode(){
  //   if(shooterMode == ShooterMode.kAuto){
  //     shooterMode = ShooterMode.kFixed;
  //   }
  //   else{
  //     shooterMode = ShooterMode.kAuto;
  //   }
  // }

  public void setHighMode(){
    mode = ShooterMode.kHigh;
  }

  public void setMidMode(){
    mode = ShooterMode.kMid;
  }

  public void setCSMode(){
    mode = ShooterMode.kCS;
  }

  public void shooterOff(){
    topController.setReference(
      0,
      CANSparkMax.ControlType.kVoltage,
      0);

    botController.setReference(
      0,
      CANSparkMax.ControlType.kVoltage,
      0);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(!SmartDashboard.containsKey("Shooter Voltage")){
      SmartDashboard.putNumber("Shooter Voltage", 4.25);
    }

    // targetPose = NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
    // dist = Math.sqrt(Math.pow(targetPose[0], 2) + Math.pow(targetPose[2], 2));

    // if (llTable.getEntry("tv").getDouble(0) != 0 && shooterMode == ShooterMode.kAuto) {
    //   dist = distFilter.calculate(dist);
    //   target = shooterLerp.interpolate(dist);
    // }
    if(mode == ShooterMode.kHigh) {
      target = 2.27;
    }
    else if (mode == ShooterMode.kMid){
      target = 1.7;
    }
    else{
      target = SmartDashboard.getNumber("Shooter Voltage", 4.25);
    }

    SmartDashboard.putNumber("Shooter Target", target);
    SmartDashboard.putBoolean("Distance Sensor", !irSensor.get());
    SmartDashboard.putString("Shooter Mode", mode.toString());
    SmartDashboard.putNumber("Top Shooter Velocity", topEncoder.getVelocity());
    SmartDashboard.putNumber("Bot Shooter Velocity", botEncoder.getVelocity());

    if(RobotContainer.backupOpController.getPOV() == 0){
      mode = ShooterMode.kHigh;
    }
    else if(RobotContainer.backupOpController.getPOV() == 90){
      mode = ShooterMode.kMid;
    }
    else if(RobotContainer.backupOpController.getPOV() == 180){
      mode = ShooterMode.kCS;
    }
  }

  public NetworkTable getLLTable(){
    return llTable;
  }

  public void changeLLPipeline(int pipeline){
    llTable.getEntry("pipeline").setNumber(pipeline);
  }
}
