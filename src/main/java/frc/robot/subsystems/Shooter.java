// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.lib.util.LerpTable;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private PearadoxSparkMax topShooter;
  private PearadoxSparkMax botShooter;
  private PearadoxSparkMax feeder;

  private SparkMaxPIDController topController;
  private SparkMaxPIDController botController;
  private SimpleMotorFeedforward shooterFeedforward = new SimpleMotorFeedforward(ShooterConstants.SHOOTER_kS, ShooterConstants.SHOOTER_kV, ShooterConstants.SHOOTER_kA);

  private DigitalInput irSensor;

  private NetworkTable llTable = NetworkTableInstance.getDefault().getTable("limelight-shooter");
  private LerpTable shooterLerp;
  private double[] targetPose;

  private enum ShooterMode{
    kAuto, kFixed
  }
  private ShooterMode shooterMode = ShooterMode.kAuto;

  private static final Shooter shooter = new Shooter();

  public static Shooter getInstance(){
    return shooter;
  }

  /** Creates a new Shooter. */
  public Shooter() {
    topShooter = new PearadoxSparkMax(ShooterConstants.TOP_SHOOTER_ID, MotorType.kBrushless, IdleMode.kBrake, 50, true,
      ShooterConstants.SHOOTER_kP, ShooterConstants.SHOOTER_kI, ShooterConstants.SHOOTER_kD,
      ShooterConstants.SHOOTER_MIN_OUTPUT, ShooterConstants.SHOOTER_MAX_OUTPUT);
    botShooter = new PearadoxSparkMax(ShooterConstants.BOT_SHOOTER_ID, MotorType.kBrushless, IdleMode.kBrake, 50, false,
      ShooterConstants.SHOOTER_kP, ShooterConstants.SHOOTER_kI, ShooterConstants.SHOOTER_kD,
      ShooterConstants.SHOOTER_MIN_OUTPUT, ShooterConstants.SHOOTER_MAX_OUTPUT);
    feeder = new PearadoxSparkMax(ShooterConstants.FEEDER_ID, MotorType.kBrushless, IdleMode.kBrake, 50, true);

    topController = topShooter.getPIDController();
    botController = botShooter.getPIDController();

    irSensor = new DigitalInput(0);

    shooterLerp = new LerpTable();
  }

  public void shooterHold(){
    topController.setReference(
    desiredState.speedMetersPerSecond,
    CANSparkMax.ControlType.kVelocity,
    0,
    shooterFeedforward.calculate(desiredState.speedMetersPerSecond));

    botController.setReference(
      desiredState.speedMetersPerSecond,
      CANSparkMax.ControlType.kVelocity,
      0,
      shooterFeedforward.calculate(desiredState.speedMetersPerSecond));
  }

  public void feederHold(){
    if(!hasCube()){
      feeder.set(0.5);
    }
    else{
      feeder.set(0);
    }
  }

  public void feederShoot(){
    feeder.set(0.7);
  }

  public boolean hasCube(){
    return !irSensor.get();
  }

  public void toggleMode(){
    if(shooterMode == ShooterMode.kAuto){
      shooterMode = ShooterMode.kFixed;
    }
    else{
      shooterMode = ShooterMode.kAuto;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    targetPose = NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);

    if(!SmartDashboard.containsKey("Shooter Speed")){
      SmartDashboard.putNumber("Shooter Speed", 0.4);
    }
    SmartDashboard.putBoolean("Distance Sensor", !irSensor.get());
  }

  public NetworkTable getLLTable(){
    return llTable;
  }

  public void changeLLPipeline(int pipeline){
    llTable.getEntry("pipeline").setNumber(pipeline);
  }
}
