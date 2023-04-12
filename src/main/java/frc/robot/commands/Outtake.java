// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class Outtake extends CommandBase {
  /** Creates a new Outtake. */
  public Outtake() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.transport, RobotContainer.intakeRollers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.transport.setHolding(false);
    if(RobotContainer.intake.isDeployed()){
      RobotContainer.intake.intakeToggle();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.transport.feederOut(-0.5);
    RobotContainer.intakeRollers.intakeOut();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.transport.setHolding(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
