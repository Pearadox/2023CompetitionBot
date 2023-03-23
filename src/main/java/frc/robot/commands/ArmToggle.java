// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class ArmToggle extends InstantCommand {
  private int armMode;

  /** Creates a new ArmToggle. */
  public ArmToggle(int armMode) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intake, RobotContainer.bigStick, RobotContainer.arm);
    this.armMode = armMode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(RobotContainer.intake.isDeployed()){
      RobotContainer.intake.intakeToggle();
    }
    if(RobotContainer.bigStick.isDeployed()){
      RobotContainer.bigStick.toggleDeploy();
    }
    
    switch(armMode){
      case 0:
        RobotContainer.arm.setZeroMode();
        break;
      case 1:
        RobotContainer.arm.setLowMode();
        break;
      case 2:
        RobotContainer.arm.setMidMode();
        break;
      case 3:
        RobotContainer.arm.setHighMode();
        break;
      case 4:
        RobotContainer.arm.setSubsMode();
        break;
      case 5:
        RobotContainer.arm.setGroundConeMode();
        break;
      default:
        RobotContainer.arm.setZeroMode();
    }
  }
}
