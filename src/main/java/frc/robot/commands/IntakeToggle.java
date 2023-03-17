// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class IntakeToggle extends InstantCommand {
  /** Creates a new IntakeToggle. */
  public IntakeToggle() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intake, RobotContainer.bigStick, RobotContainer.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(RobotContainer.bigStick.isDeployed()){
      RobotContainer.bigStick.toggleDeploy();
    }
    if(RobotContainer.arm.isDeployed()){
      RobotContainer.arm.setZeroMode();
    }
    RobotContainer.intake.intakeToggle();
  }
}
