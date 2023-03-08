// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants;

public final class Autos {
  private static PIDController frontController = new PIDController(SwerveConstants.AUTO_kP_FRONT, 0, 0);
  private static PIDController sideController = new PIDController(SwerveConstants.AUTO_kP_SIDE, 0, 0);
  private static PIDController turnController = new PIDController(SwerveConstants.AUTO_kP_TURN, 0, 0);

  /** Example static factory for an autonomous command. */
  public static CommandBase c1C0_M_Bal() {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath("c1C0_M_Bal", SwerveConstants.AUTO_DRIVE_MAX_SPEED / 2.25, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION);

    PPSwerveControllerCommand driveOnCS = makeSwerveControllerCommand(trajectory);

    return new SequentialCommandGroup(
      new InstantCommand(() -> RobotContainer.drivetrain.resetOdometry(getInitialPose(trajectory))),
      new InstantCommand(() -> RobotContainer.drivetrain.setAllMode(true)),
      new InstantCommand(() -> RobotContainer.shooter.setHighMode()),
      new Shoot().withTimeout(2),
      driveOnCS,
      new AutoBalance().until(() -> (Math.abs(RobotContainer.drivetrain.getRoll()) < 2.0 && Math.abs(RobotContainer.drivetrain.getPitch()) < 2.0)),
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules())
    );
  }

  public static CommandBase c2C0_NC_Bal() {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("c2C0_NC_Bal", 
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED / 1.5, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION));

    PPSwerveControllerCommand driveToCube1 = makeSwerveControllerCommand(pathGroup.get(0));
    PPSwerveControllerCommand driveOnCS = makeSwerveControllerCommand(pathGroup.get(1));
    
    return Commands.sequence(
      new InstantCommand(() -> RobotContainer.drivetrain.resetOdometry(getInitialPose(pathGroup.get(0)))),
      new InstantCommand(() -> RobotContainer.drivetrain.setAllMode(true)),
      new InstantCommand(() -> RobotContainer.shooter.setHighMode()),
      new WaitCommand(1),
      new Shoot().withTimeout(1),
      new InstantCommand(() -> RobotContainer.shooter.detectCube(false)),
      new InstantCommand(() -> RobotContainer.intake.intakeToggle()),
      new InstantCommand(() -> RobotContainer.shooter.setCSMode()),
      driveToCube1,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules()),
      new Shoot().withTimeout(2),
      new InstantCommand(() -> RobotContainer.shooter.detectCube(false)),
      driveOnCS,
      new AutoBalance(),
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules())
    );
  }

  public static CommandBase c1C1_NC_Bal() {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("c1C1_NC_Bal",
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED / 2.0, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION), 
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED / 1.5, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION));

    PPSwerveControllerCommand scoreCone1 =  makeSwerveControllerCommand(pathGroup.get(0));
    PPSwerveControllerCommand driveToCube1 =  makeSwerveControllerCommand(pathGroup.get(1));
    PPSwerveControllerCommand driveOnCS =  makeSwerveControllerCommand(pathGroup.get(2));
    
    return new SequentialCommandGroup(
      new InstantCommand(() -> RobotContainer.drivetrain.resetOdometry(getInitialPose(pathGroup.get(0)))),
      new InstantCommand(() -> RobotContainer.drivetrain.setAllMode(true)),
      new InstantCommand(() -> RobotContainer.arm.setHighMode()),
      new InstantCommand(() -> RobotContainer.intake.intakeToggle()),
      scoreCone1,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules()),
      new RunCommand(() -> RobotContainer.arm.intakeOut()).withTimeout(0.5),
      new InstantCommand(() -> RobotContainer.arm.intakeIn()),
      driveToCube1.alongWith(
        new WaitCommand(0.5)
        .andThen(new InstantCommand(() -> RobotContainer.arm.setZeroMode()))
      ),
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules()),
      new InstantCommand(() -> RobotContainer.shooter.setCSMode()),
      new InstantCommand(() -> RobotContainer.intake.intakeToggle()),
      new InstantCommand(() -> RobotContainer.bigStick.toggleDeploy()),
      new Shoot().withTimeout(2),
      new InstantCommand(() -> RobotContainer.bigStick.toggleDeploy()),
      driveOnCS,
      new AutoBalance().until(() -> (Math.abs(RobotContainer.drivetrain.getRoll()) < 2.0 && Math.abs(RobotContainer.drivetrain.getPitch()) < 2.0)),
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules())
    );
  }

  public static CommandBase c1C1_C_Bal() {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("c1C1_C_Bal",
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED / 2.0, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION), 
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED / 2.0, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED / 1.75, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION));

    PPSwerveControllerCommand scoreCone1 =  makeSwerveControllerCommand(pathGroup.get(0));
    PPSwerveControllerCommand driveOut =  makeSwerveControllerCommand(pathGroup.get(1));
    PPSwerveControllerCommand driveOverCP =  makeSwerveControllerCommand(pathGroup.get(2));
    PPSwerveControllerCommand driveToCube1 =  makeSwerveControllerCommand(pathGroup.get(3));
    PPSwerveControllerCommand driveOnCS =  makeSwerveControllerCommand(pathGroup.get(4));
    
    return new SequentialCommandGroup(
      new InstantCommand(() -> RobotContainer.drivetrain.resetOdometry(getInitialPose(pathGroup.get(0)))),
      new InstantCommand(() -> RobotContainer.drivetrain.setAllMode(true)),
      new InstantCommand(() -> RobotContainer.arm.setHighMode()),
      scoreCone1,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules()),
      new WaitCommand(1),
      driveOut.alongWith(
        new InstantCommand(() -> RobotContainer.arm.setZeroMode())
        .andThen(new InstantCommand(() -> RobotContainer.intake.intakeToggle()))
      ),
      driveOverCP,
      driveToCube1,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules()),
      new WaitCommand(1),
      driveOnCS,
      new AutoBalance().until(() -> (Math.abs(RobotContainer.drivetrain.getRoll()) < 2.0 && Math.abs(RobotContainer.drivetrain.getPitch()) < 2.0)),
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules())
    );
  }

  public static CommandBase testAuto(){
    return new SequentialCommandGroup(
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules()),
      new InstantCommand(() -> RobotContainer.intake.intakeToggle())
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  private static PPSwerveControllerCommand makeSwerveControllerCommand(PathPlannerTrajectory traj){
    turnController.enableContinuousInput(-Math.PI, Math.PI);
    return new PPSwerveControllerCommand(
      traj, 
      RobotContainer.drivetrain::getPose, 
      SwerveConstants.DRIVE_KINEMATICS,
      frontController, 
      sideController,
      turnController, 
      RobotContainer.drivetrain::setModuleStates,
      true,
      RobotContainer.drivetrain);
  }

  private static Pose2d getInitialPose(PathPlannerTrajectory traj){
    PathPlannerState initialState = DriverStation.getAlliance().equals(Alliance.Red) ? 
    PathPlannerTrajectory.transformStateForAlliance(traj.getInitialState(), Alliance.Red) :
    traj.getInitialState();

    return new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation);
  }
}
