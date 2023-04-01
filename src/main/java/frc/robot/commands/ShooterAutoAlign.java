// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;

public class ShooterAutoAlign extends CommandBase {
  private double targetAngle;

  /** Creates a new AutoAlign. */
  public ShooterAutoAlign() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetAngle = RobotContainer.shooter.getTargetAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.shooter.hasLLTarget()){
      RobotContainer.drivetrain.turnToHeading(
        targetAngle, 
        new Translation2d(
          VisionConstants.ROBOT_TO_SHOOTER_LL.getX(),
          VisionConstants.ROBOT_TO_SHOOTER_LL.getY()
        ));
    }
    else{
      RobotContainer.drivetrain.swerveDrive(
        -RobotContainer.driverController.getLeftY(), 
        -RobotContainer.driverController.getLeftX(), 
        -RobotContainer.driverController.getRightX(),
        !RobotContainer.driverController.getRawButton(XboxController.Button.kB.value),
        new Translation2d(),
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
