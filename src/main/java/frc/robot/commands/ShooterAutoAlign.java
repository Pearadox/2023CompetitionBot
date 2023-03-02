// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants;

public class ShooterAutoAlign extends CommandBase {
  int r;
  int c;

  /** Creates a new AutoAlign. */
  public ShooterAutoAlign(int r, int c) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drivetrain);
    this.r = r;
    this.c = c;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(DriverStation.getAlliance().equals(Alliance.Red)){
      if((r == 7 && c == 1) || (r == 8 && c == 1)){
        RobotContainer.shooter.changeLLPipeline(1);
      }
      else if((r == 7 && c == 4) || (r == 8 && c == 4)){
        RobotContainer.shooter.changeLLPipeline(2);
      }
      else if((r == 7 && c == 7) || (r == 8 && c == 7)){
        RobotContainer.shooter.changeLLPipeline(3);
      }
    }
    else if(DriverStation.getAlliance().equals(Alliance.Blue)){
      if(DriverStation.getAlliance().equals(Alliance.Red)){
        if((r == 7 && c == 1) || (r == 8 && c == 1)){
          RobotContainer.shooter.changeLLPipeline(6);
        }
        else if((r == 7 && c == 4) || (r == 8 && c == 4)){
          RobotContainer.shooter.changeLLPipeline(7);
        }
        else if((r == 7 && c == 7) || (r == 8 && c == 7)){
          RobotContainer.shooter.changeLLPipeline(8);
        }
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = RobotContainer.shooter.getLLTable().getEntry("tx").getDouble(0);
    double hasTarget = RobotContainer.shooter.getLLTable().getEntry("tv").getDouble(0);
    if(hasTarget != 0){
      RobotContainer.drivetrain.swerveDrive(0, 0, Math.signum(error) * SwerveConstants.kS_PERCENT + SwerveConstants.kP_PERCENT * error, 0, true, false, false);
    }
    else{
      RobotContainer.drivetrain.swerveDrive(
        -RobotContainer.driverController.getLeftY(), 
        -RobotContainer.driverController.getLeftX(), 
        RobotContainer.driverController.getRightX(),
        -RobotContainer.driverController.getRightY(),
        !RobotContainer.driverController.getRawButton(XboxController.Button.kB.value),
        true,
        true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
