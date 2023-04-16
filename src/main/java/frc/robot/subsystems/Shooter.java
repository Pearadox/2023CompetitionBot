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

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.lib.util.LerpTable;
import frc.robot.RobotContainer;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;

public class Shooter extends SubsystemBase {
  private PearadoxSparkMax topShooter;
  private PearadoxSparkMax botShooter;

  private RelativeEncoder topEncoder;
  private RelativeEncoder botEncoder;

  private SparkMaxPIDController topController;
  private SparkMaxPIDController botController;
  // private SimpleMotorFeedforward topShooterFeedforward = new SimpleMotorFeedforward(ShooterConstants.TOP_SHOOTER_kS, ShooterConstants.TOP_SHOOTER_kV, ShooterConstants.TOP_SHOOTER_kA);
  // private SimpleMotorFeedforward botShooterFeedforward = new SimpleMotorFeedforward(ShooterConstants.BOT_SHOOTER_kS, ShooterConstants.BOT_SHOOTER_kV, ShooterConstants.BOT_SHOOTER_kA);

  private NetworkTable llTable = NetworkTableInstance.getDefault().getTable(VisionConstants.SHOOTER_LL_NAME);
  private LerpTable shooterLerp;
  private double target;
  private double[] cameraPose = new double[6];

  private int r = 0;
  private int c = 0;
  private double tz = 0;
  private double tx = 0;
  private double dist = 0;
  private double targetAngle = 0;
  private MedianFilter distFilter = new MedianFilter(5);

  private enum ShooterMode{
    kAuto, kHigh, kMid, kCS, kTest
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

    shooterLerp = new LerpTable();

    //SHOOTER LOOKUP TABLE: (distance, voltage)
    shooterLerp.addPoint(0, 0);
    // 2.0 0.75
    // 2.2 0.918
    // 2.45 1.179
    // 2.9 1.348
    // bottom spinning +0.5 V

    llTable.getEntry("pipeline").setNumber(0);
  }

  public void shooterHold(){
    if(mode == ShooterMode.kCS){
      topController.setReference(
        target + 2,
        CANSparkMax.ControlType.kVoltage,
        0);
  
      botController.setReference(
        //target-2,
        0,
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
      target + 0.5,
      CANSparkMax.ControlType.kVoltage,
      0);
    }
  }

  // public void toggleMode(){
  //   if(shooterMode == ShooterMode.kAuto){
  //     shooterMode = ShooterMode.kFixed;
  //   }
  //   else{
  //     shooterMode = ShooterMode.kAuto;
  //   }
  // }

  public void setAutoMode(){
    mode = ShooterMode.kAuto;
  }

  public void setHighMode(){
    mode = ShooterMode.kHigh;
  }

  public void setMidMode(){
    mode = ShooterMode.kMid;
  }

  public void setCSMode(){
    mode = ShooterMode.kCS;
  }

  public void setTestMode(){
    mode = ShooterMode.kTest;
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
      SmartDashboard.putNumber("Shooter Voltage", 4);
    }

    // setPipeline(c);
    if (hasLLTarget() && mode == ShooterMode.kAuto) {
      cameraPose = llTable.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
      tz = -cameraPose[2];
      tx = cameraPose[0];
      SmartDashboard.putNumber("Shooter tx", tx);
      SmartDashboard.putNumber("Shooter tz", tz);
      Logger.getInstance().recordOutput("Shooter/tx", tx);
      Logger.getInstance().recordOutput("Shooter/tz", tz);
      calculateTx(r, c); //adjust tx and tz based on target node
      calculateTz(r);
      dist = Math.hypot(Math.abs(tx), Math.abs(tz));
    
      dist = distFilter.calculate(dist);
      target = shooterLerp.interpolate(dist);

      setTargetAngle();
      SmartDashboard.putNumber("Target Angle", targetAngle);
      Logger.getInstance().recordOutput("Shooter/Target Angle", targetAngle);
    }
    else if(mode == ShooterMode.kHigh) {
      target = 2.1;
    }
    else if (mode == ShooterMode.kMid){
      target = 1;
    }
    else{
      target = SmartDashboard.getNumber("Shooter Voltage", 4);
      Logger.getInstance().recordOutput("Shooter/Shooter Voltage", SmartDashboard.getNumber("Shooter Voltage", 4));
    }

    SmartDashboard.putNumber("Shooter Target", target);
    SmartDashboard.putString("Shooter Mode", mode.toString());
    SmartDashboard.putNumber("Target Node R", r);
    SmartDashboard.putNumber("Target Node C", c);
    SmartDashboard.putNumber("Top Shooter Velocity", topEncoder.getVelocity());
    SmartDashboard.putNumber("Bot Shooter Velocity", botEncoder.getVelocity());

    Logger.getInstance().recordOutput("Shooter/Target Voltage", target);
    Logger.getInstance().recordOutput("Shooter/Mode", mode.toString());
    Logger.getInstance().recordOutput("Shooter/Target Node R", r);
    Logger.getInstance().recordOutput("Shooter/Target Node C", c);
    Logger.getInstance().recordOutput("Shooter/Top Velocity", topEncoder.getVelocity());
    Logger.getInstance().recordOutput("Shooter/Bottom Velocity", botEncoder.getVelocity());

    if(RobotContainer.opController.getPOV() == 0){
      mode = ShooterMode.kHigh;
    }
    else if(RobotContainer.opController.getPOV() == 90){
      mode = ShooterMode.kMid;
    }
    else if(RobotContainer.opController.getPOV() == 180){
      mode = ShooterMode.kCS;
    }
  }
  
  public NetworkTable getLLTable(){                                                                                                    
    return llTable;
  }

  public boolean hasLLTarget(){
    return llTable.getEntry("tv").getDouble(0) != 0;
  }

  public double getTa(){
    return llTable.getEntry("ta").getDouble(0);
  }

  public void changeLLPipeline(int pipeline){
    llTable.getEntry("pipeline").setNumber(pipeline);
  }

  public void setTargetNode(int r, int c){
    this.r = r;
    this.c = c;
  }

  public void setPipeline(int c){
    if(DriverStation.getAlliance().equals(Alliance.Red)){
      if(c == 0 || c == 1 || c == 2){
        changeLLPipeline(1);
      }
      if(c == 3 || c == 4 || c == 5){
        changeLLPipeline(2);
      }
      if(c == 6 || c == 7 || c == 8){
        changeLLPipeline(3);
      }
    }
    else if(DriverStation.getAlliance().equals(Alliance.Blue)){
      if(c == 0 || c == 1 || c == 2){
        changeLLPipeline(6);
      }
      if(c == 3 || c == 4 || c == 5){
        changeLLPipeline(7);
      }
      if(c == 6 || c == 7 || c == 8){
        changeLLPipeline(8);
      }
    }
  }

  public void calculateTz(int r){
    switch(r){
      case 0:
        tz -= FieldConstants.APRIL_TAG_TO_HYBRID;
        break;
      case 1:
        tz += FieldConstants.APRIL_TAG_TO_MID;
        break;
      case 2:
        tz += FieldConstants.APRIL_TAG_TO_HIGH;
        break;
      default:
        tz -= FieldConstants.APRIL_TAG_TO_HYBRID;
    }
  }

  public void calculateTx(int r, int c){
    if(r == 0){
      if(c == 0){
        tx -= FieldConstants.HYBRID_TO_OUTER_HYBRID;
      }
      else if(c == 8){
        tx += FieldConstants.HYBRID_TO_OUTER_HYBRID;
      }
      else if(c == 2 || c == 5){
        tx += FieldConstants.HYBRID_TO_INNER_HYBRID;
      }
      else if(c == 3 || c == 6){
        tx -= FieldConstants.HYBRID_TO_INNER_HYBRID;
      }
    }
  }

  public void setTargetAngle(){
    if(tz != 0){
      targetAngle = Math.toDegrees(Math.atan(tx/tz));
    }
    else{
      targetAngle = 0;
    }
  }

  public double getTargetAngle(){
    return targetAngle;
  }

  public boolean isHighMode(){
    return mode == ShooterMode.kHigh;
  }

  public boolean isMidMode(){
    return mode == ShooterMode.kMid;
  }

  public boolean isCSMode(){
    return mode == ShooterMode.kCS;
  }
}
