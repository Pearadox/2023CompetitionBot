// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase OneM_Balance() {
    PIDController frontController = new PIDController(SwerveConstants.AUTO_kP_FRONT, 0, 0);
    PIDController sideController = new PIDController(SwerveConstants.AUTO_kP_SIDE, 0, 0);
    PIDController turnController = new PIDController(SwerveConstants.AUTO_kP_TURN, 0, 0);
    turnController.enableContinuousInput(-Math.PI, Math.PI);

    PathPlannerTrajectory trajectory = PathPlanner.loadPath("OneM_Balance", SwerveConstants.AUTO_DRIVE_MAX_SPEED / 2.25, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION);

    PPSwerveControllerCommand command = new PPSwerveControllerCommand(
      trajectory, 
      RobotContainer.drivetrain::getPose, 
      SwerveConstants.DRIVE_KINEMATICS,
      frontController, 
      sideController, 
      turnController, 
      RobotContainer.drivetrain::setModuleStates,
      true,
      RobotContainer.drivetrain);

    PathPlannerState initialState = DriverStation.getAlliance().equals(Alliance.Red) ? 
      PathPlannerTrajectory.transformStateForAlliance(trajectory.getInitialState(), Alliance.Red) :
      trajectory.getInitialState();

    Pose2d initialPose = new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation);
    
    return new SequentialCommandGroup(
      new InstantCommand(() -> RobotContainer.drivetrain.resetOdometry(initialPose)),
      command,
      new AutoBalance().until(() -> (Math.abs(RobotContainer.drivetrain.getRoll()) < 2.0 && Math.abs(RobotContainer.drivetrain.getPitch()) < 2.0)),
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules())
    );
  }

  public static CommandBase TwoNC_Balance() {
    PIDController frontController = new PIDController(SwerveConstants.AUTO_kP_FRONT, 0, 0);
    PIDController sideController = new PIDController(SwerveConstants.AUTO_kP_SIDE, 0, 0);
    PIDController turnController = new PIDController(SwerveConstants.AUTO_kP_TURN, 0, 0);
    turnController.enableContinuousInput(-Math.PI, Math.PI);

    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("TwoNC_Balance", SwerveConstants.AUTO_DRIVE_MAX_SPEED / 1.75, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION);

    PPSwerveControllerCommand command1 = new PPSwerveControllerCommand(
      pathGroup.get(0), 
      RobotContainer.drivetrain::getPose, 
      SwerveConstants.DRIVE_KINEMATICS,
      frontController, 
      sideController, 
      turnController, 
      RobotContainer.drivetrain::setModuleStates,
      true,
      RobotContainer.drivetrain);

    PPSwerveControllerCommand command2 = new PPSwerveControllerCommand(
      pathGroup.get(1), 
      RobotContainer.drivetrain::getPose, 
      SwerveConstants.DRIVE_KINEMATICS,
      frontController, 
      sideController, 
      turnController, 
      RobotContainer.drivetrain::setModuleStates,
      true,
      RobotContainer.drivetrain);

    PathPlannerState initialState = DriverStation.getAlliance().equals(Alliance.Red) ? 
      PathPlannerTrajectory.transformStateForAlliance(pathGroup.get(0).getInitialState(), Alliance.Red) :
      pathGroup.get(0).getInitialState();

    Pose2d initialPose = new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation);
    
    return new SequentialCommandGroup(
      new InstantCommand(() -> RobotContainer.drivetrain.resetOdometry(initialPose)),
      new InstantCommand(() -> RobotContainer.intake.intakeToggle()),
      command1,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules()),
      new WaitCommand(1),
      command2,
      new AutoBalance().until(() -> (Math.abs(RobotContainer.drivetrain.getRoll()) < 2.0 && Math.abs(RobotContainer.drivetrain.getPitch()) < 2.0)),
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules())
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
