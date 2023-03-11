// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.DecimalFormat;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants;

public class Drivetrain extends SubsystemBase {
  private SwerveModule leftFront = new SwerveModule(
    SwerveConstants.LEFT_FRONT_DRIVE_ID, 
    SwerveConstants.LEFT_FRONT_TURN_ID, 
    false, 
    true, 
    SwerveConstants.LEFT_FRONT_CANCODER_ID, 
    SwerveConstants.LEFT_FRONT_OFFSET, 
    false);

  private SwerveModule rightFront = new SwerveModule(
    SwerveConstants.RIGHT_FRONT_DRIVE_ID, 
    SwerveConstants.RIGHT_FRONT_TURN_ID, 
    false, 
    true, 
    SwerveConstants.RIGHT_FRONT_CANCODER_ID, 
    SwerveConstants.RIGHT_FRONT_OFFSET, 
    false);

  private SwerveModule leftBack = new SwerveModule(
    SwerveConstants.LEFT_BACK_DRIVE_ID, 
    SwerveConstants.LEFT_BACK_TURN_ID, 
    false, 
    true, 
    SwerveConstants.LEFT_BACK_CANCODER_ID, 
    SwerveConstants.LEFT_BACK_OFFSET, 
    false);

  private SwerveModule rightBack = new SwerveModule(
    SwerveConstants.RIGHT_BACK_DRIVE_ID, 
    SwerveConstants.RIGHT_BACK_TURN_ID, 
    false, 
    true, 
    SwerveConstants.RIGHT_BACK_CANCODER_ID, 
    SwerveConstants.RIGHT_BACK_OFFSET, 
    false);

  private SlewRateLimiter frontLimiter = new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ACCELERATION);
  private SlewRateLimiter sideLimiter = new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ACCELERATION);
  private SlewRateLimiter turnLimiter = new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION);

  private Pigeon2 gyro = new Pigeon2(SwerveConstants.PIGEON_ID);
  private double rates[] = new double[3];

  private SwerveDriveOdometry odometry = new SwerveDriveOdometry(SwerveConstants.DRIVE_KINEMATICS, new Rotation2d(0), getModulePositions());

  private enum DriveMode{
    kNormal, kSubs, kGrid
  }

  DriveMode mode = DriveMode.kNormal;

  private static final Drivetrain drivetrain = new Drivetrain();

  public static Drivetrain getInstance(){
    return drivetrain;
  }

  /** Creates a new SwerveDrivetrain. */
  public Drivetrain() {
    new Thread(() -> {
      try{
        Thread.sleep(1000);
        zeroHeading();
      }
      catch(Exception e){}
    }).start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(getHeadingRotation2d(), getModulePositions());
    gyro.getRawGyro(rates);
    SmartDashboard.putNumber("Robot Angle", getHeading());
    SmartDashboard.putNumber("Robot Pitch", getPitch());
    SmartDashboard.putNumber("Robot Roll", getRoll());
    SmartDashboard.putString("Pose", getPose().toString());
    SmartDashboard.putString("Angular Speed", new DecimalFormat("#.00").format((rates[2] / 180)) + "pi rad/s");
  }

  public void swerveDrive(double frontSpeed, double sideSpeed, double turnX, double turnY, 
    boolean fieldOriented, boolean headingControl, boolean deadband){
    if(deadband){
      frontSpeed = Math.abs(frontSpeed) > 0.1 ? frontSpeed : 0;
      sideSpeed = Math.abs(sideSpeed) > 0.1 ? sideSpeed : 0;
      turnX = Math.abs(turnX) > 0.1 ? turnX : 0;
      turnY = Math.abs(turnY) > 0.1 ? turnY : 0;
    }

    double turnSpeed;
    if(headingControl){
      if(turnX == 0 && turnY == 0){
        turnSpeed = 0;
      }
      else{
        double error = getJoystickAngle(turnX, turnY) - getHeading();
    
        if(error > 180) {
          error -= 360;
        }
        else if(error < -180){
          error += 360;
        }
    
        if(Math.abs(error) > 1){
          turnSpeed = Math.signum(error) * SwerveConstants.kS_PERCENT + SwerveConstants.kP_PERCENT * error;
        }
        else{
          turnSpeed = 0;
        }
      }
    }
    else{
      turnSpeed = -turnX;
    }

    frontSpeed = RobotContainer.driverController.getLeftTriggerAxis() > 0.9 ? frontSpeed * 0.5 : frontSpeed;
    sideSpeed = RobotContainer.driverController.getLeftTriggerAxis() > 0.9 ? sideSpeed * 0.5 : sideSpeed;
    turnSpeed = RobotContainer.driverController.getLeftTriggerAxis() > 0.9 ? turnSpeed * 0.5 : turnSpeed;

    frontSpeed = frontLimiter.calculate(frontSpeed) * SwerveConstants.TELE_DRIVE_MAX_SPEED;
    sideSpeed = sideLimiter.calculate(sideSpeed) * SwerveConstants.TELE_DRIVE_MAX_SPEED;
    turnSpeed = turnLimiter.calculate(turnSpeed) * SwerveConstants.TELE_DRIVE_MAX_ANGULAR_SPEED;

    ChassisSpeeds chassisSpeeds;
    if(fieldOriented){
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(frontSpeed, sideSpeed, turnSpeed, getHeadingRotation2d());
    }
    else{
      chassisSpeeds = new ChassisSpeeds(frontSpeed, sideSpeed, turnSpeed);
    }

    SwerveModuleState[] moduleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

    setModuleStates(moduleStates);
  }
  
  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose){
    odometry.resetPosition(getHeadingRotation2d(), getModulePositions(), pose);
  }


  public void setAllMode(boolean brake){
    if(brake){
      leftFront.setBrake(true);
      rightFront.setBrake(true);
      leftBack.setBrake(true);
      rightBack.setBrake(true);
    }
    else{
      leftFront.setBrake(false);
      rightFront.setBrake(false);
      leftBack.setBrake(false);
      rightBack.setBrake(false);
    }
  }

  public void resetAllEncoders(){
    leftFront.resetEncoders();
    rightFront.resetEncoders();
    leftBack.resetEncoders();
    rightBack.resetEncoders();
  }

  public void zeroHeading(){
    gyro.setYaw(0);
  }

  public void setHeading(double heading){
    gyro.setYaw(heading);
  }

  public double getHeading(){
    return Math.IEEEremainder(gyro.getYaw(), 360);
  }

  public Rotation2d getHeadingRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public double getRoll(){
    return gyro.getRoll() + 0.92;
  }

  public double getPitch(){
    return gyro.getPitch() + 0.13;
  }

  public void formX(){
    leftFront.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    rightFront.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    leftBack.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rightBack.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void stopModules(){
    leftFront.stop();
    leftBack.stop();
    rightFront.stop();
    rightBack.stop();
  }

  public void setModuleStates(SwerveModuleState[] moduleStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.DRIVETRAIN_MAX_SPEED);
    leftFront.setDesiredState(moduleStates[0]);
    rightFront.setDesiredState(moduleStates[1]);
    leftBack.setDesiredState(moduleStates[2]);
    rightBack.setDesiredState(moduleStates[3]);
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = leftFront.getPosition();
    positions[1] = rightFront.getPosition();
    positions[2] = leftBack.getPosition();
    positions[3] = rightBack.getPosition();
    return positions;
  } 

  public double getJoystickAngle(double turnX, double turnY){
    double targetAngle;

    if(turnX == 0 && turnY > 0){
      targetAngle = -90.0;
    }
    else if(turnX == 0 && turnY < 0){
      targetAngle = 90.0;
    }
    else{
      targetAngle = Math.atan(turnY/turnX) * (180 / Math.PI);
    }
    
    targetAngle -= 90.0;
      
    if(turnX > 0 && turnY > 0){
      targetAngle -= 180.0;
    }
    else if(turnX > 0 && turnY <= 0){
      targetAngle += 180.0;
    }
  
    if(targetAngle + 180 > 180){
      targetAngle -= 180.0;
    }
    else{
      targetAngle += 180.0;
    }

    return targetAngle;
  }

  public DriveMode getDriveMode(){
    return mode;
  }

  public DriveMode getNormalMode(){
    return DriveMode.kNormal;
  }

  public DriveMode getSubsMode(){
    return DriveMode.kSubs;
  }

  public DriveMode getGridMode(){
    return DriveMode.kGrid;
  }

  public void setGridMode(){
    mode = DriveMode.kGrid;
  }

  public void setSubsMode(){
    mode = DriveMode.kSubs;
  }

  public void setNormalMode(){
    mode = DriveMode.kNormal;
  }
}
