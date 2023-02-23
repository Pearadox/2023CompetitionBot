// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

  private DigitalInput irSensor;

  private LerpTable shooterLerp;

  private static final Shooter shooter = new Shooter();

  public static Shooter getInstance(){
    return shooter;
  }

  /** Creates a new Shooter. */
  public Shooter() {
    topShooter = new PearadoxSparkMax(ShooterConstants.TOP_SHOOTER_ID, MotorType.kBrushless, IdleMode.kBrake, 50, true);
    botShooter = new PearadoxSparkMax(ShooterConstants.BOT_SHOOTER_ID, MotorType.kBrushless, IdleMode.kBrake, 50, false);
    feeder = new PearadoxSparkMax(ShooterConstants.FEEDER_ID, MotorType.kBrushless, IdleMode.kBrake, 50, true);

    topController = topShooter.getPIDController();
    botController = botShooter.getPIDController();

    irSensor = new DigitalInput(0);

    shooterLerp = new LerpTable();
  }

  public void shooterHold(){
    topShooter.set(SmartDashboard.getNumber("Shooter Speed", 0.4));
    botShooter.set(SmartDashboard.getNumber("Shooter Speed", 0.4));
      feeder.set(0.5);
  }

  public boolean hasCube(){
    return !irSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(!SmartDashboard.containsKey("Shooter Speed")){
      SmartDashboard.putNumber("Shooter Speed", 0.4);
    }
    SmartDashboard.putBoolean("Distance Sensor", !irSensor.get());
  }
}
