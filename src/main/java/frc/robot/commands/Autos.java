// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
      new InstantCommand(() -> RobotContainer.drivetrain.setAllIdleMode(true)),
      new InstantCommand(() -> RobotContainer.shooter.setHighMode()),
      new WaitCommand(0.75),
      new Shoot().withTimeout(0.5),
      driveOnCS,
      new AutoBalance(),
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules())
    );
  }

  public static CommandBase c2RC0_M_Bal() {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("c2RC0_M_Bal", 
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED / 1.75, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED / 1.75, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION));

    PPSwerveControllerCommand command = makeSwerveControllerCommand(pathGroup.get(0));
    PPSwerveControllerCommand command2 = makeSwerveControllerCommand(pathGroup.get(1));
    PPSwerveControllerCommand command3 = makeSwerveControllerCommand(pathGroup.get(2));

    FollowPathWithEvents driveOverCS = new FollowPathWithEvents(
      command,
      pathGroup.get(0).getMarkers(),
      RobotContainer.eventMap
    );

    FollowPathWithEvents driveToCube1 = new FollowPathWithEvents(
      command2,
      pathGroup.get(1).getMarkers(),
      RobotContainer.eventMap
    );

    FollowPathWithEvents driveOnCS = new FollowPathWithEvents(
      command3,
      pathGroup.get(2).getMarkers(),
      RobotContainer.eventMap
    );
    
    return Commands.sequence(
      new InstantCommand(() -> RobotContainer.drivetrain.resetOdometry(getInitialPose(pathGroup.get(0)))),
      new InstantCommand(() -> RobotContainer.drivetrain.setAllIdleMode(true)),
      new InstantCommand(() -> RobotContainer.shooter.setHighMode()),
      new WaitCommand(0.75),
      new Shoot().withTimeout(0.5),
      new InstantCommand(() -> RobotContainer.shooter.setCSMode()),
      driveOverCS,
      driveToCube1,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules()),
      driveOnCS,
      new AutoBalance(),
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules()),
      new Shoot().withTimeout(0.75)
    );
  }

  public static CommandBase c2C0_NC() {
    List<PathPlannerTrajectory> pathGroup = DriverStation.getAlliance().equals(Alliance.Blue) ? 
    PathPlanner.loadPathGroup("c2C0_NC_Blue", 
    new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
    new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION)) :
    PathPlanner.loadPathGroup("c2C0_NC_Red", 
    new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
    new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION));

    PPSwerveControllerCommand driveToCube1 = makeSwerveControllerCommand(pathGroup.get(0));
    PPSwerveControllerCommand driveToLoadZone = makeSwerveControllerCommand(pathGroup.get(1));
    
    return Commands.sequence(
      new InstantCommand(() -> RobotContainer.drivetrain.resetOdometry(getInitialPose(pathGroup.get(0)))),
      new InstantCommand(() -> RobotContainer.drivetrain.setAllIdleMode(true)),
      new InstantCommand(() -> RobotContainer.shooter.setHighMode()),
      new WaitCommand(0.75),
      new Shoot().withTimeout(0.5),
      new InstantCommand(() -> RobotContainer.intake.intakeToggle()),
      new InstantCommand(() -> RobotContainer.shooter.setMidMode()),
      driveToCube1,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules()),
      new Shoot().withTimeout(0.5),
      new InstantCommand(() -> RobotContainer.intake.intakeToggle()),
      driveToLoadZone,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules())
    );
  }

  public static CommandBase c2C0_NC_Bal() {
    List<PathPlannerTrajectory> pathGroup = DriverStation.getAlliance().equals(Alliance.Red) ? 
      PathPlanner.loadPathGroup("c2C0_NC_Bal_Red", 
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED / 1.75, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION)) :
      PathPlanner.loadPathGroup("c2C0_NC_Bal_Blue", 
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED / 1.75, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION));

    PPSwerveControllerCommand driveToCube1 = makeSwerveControllerCommand(pathGroup.get(0));
    PPSwerveControllerCommand driveOnCS = makeSwerveControllerCommand(pathGroup.get(1));
    
    return Commands.sequence(
      new InstantCommand(() -> RobotContainer.drivetrain.resetOdometry(getInitialPose(pathGroup.get(0)))),
      new InstantCommand(() -> RobotContainer.drivetrain.setAllIdleMode(true)),
      new InstantCommand(() -> RobotContainer.shooter.setHighMode()),
      new WaitCommand(0.75),
      new Shoot().withTimeout(0.5),
      new InstantCommand(() -> RobotContainer.intake.intakeToggle()),
      new InstantCommand(() -> RobotContainer.shooter.setCSMode()),
      driveToCube1,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules()),
      new Shoot().withTimeout(0.5),
      driveOnCS,
      new AutoBalance(),
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules())
    );
  }

  public static CommandBase c2C0_CF() {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("c2C0_CF", 
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION));

    PPSwerveControllerCommand command = makeSwerveControllerCommand(pathGroup.get(0));

    FollowPathWithEvents driveToCube1 = new FollowPathWithEvents(
      command,
      pathGroup.get(0).getMarkers(),
      RobotContainer.eventMap
    );
    
    return Commands.sequence(
      new InstantCommand(() -> RobotContainer.drivetrain.resetOdometry(getInitialPose(pathGroup.get(0)))),
      new InstantCommand(() -> RobotContainer.drivetrain.setAllIdleMode(true)),
      new InstantCommand(() -> RobotContainer.shooter.setCSMode()),
      new WaitCommand(0.75),
      new Shoot().withTimeout(0.5),
      new InstantCommand(() -> RobotContainer.intake.intakeToggle()),
      new InstantCommand(() -> RobotContainer.shooter.setCSMode()),
      driveToCube1,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules()),
      new InstantCommand(() -> RobotContainer.bigStick.toggleDeploy()),
      new Shoot().withTimeout(0.75),
      new InstantCommand(() -> RobotContainer.bigStick.toggleDeploy())
    );
  }

  public static CommandBase c2C0_CF_Bal() {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("c2C0_CF_Bal", 
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED / 1.75, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION));

    PPSwerveControllerCommand command = makeSwerveControllerCommand(pathGroup.get(0));
    PPSwerveControllerCommand driveOnCS = makeSwerveControllerCommand(pathGroup.get(1));

    FollowPathWithEvents driveToCube1 = new FollowPathWithEvents(
      command,
      pathGroup.get(0).getMarkers(),
      RobotContainer.eventMap
    );
    
    return Commands.sequence(
      new InstantCommand(() -> RobotContainer.drivetrain.resetOdometry(getInitialPose(pathGroup.get(0)))),
      new InstantCommand(() -> RobotContainer.drivetrain.setAllIdleMode(true)),
      new InstantCommand(() -> RobotContainer.shooter.setCSMode()),
      new WaitCommand(0.75),
      new Shoot().withTimeout(0.5),
      new InstantCommand(() -> RobotContainer.intake.intakeToggle()),
      new InstantCommand(() -> RobotContainer.shooter.setCSMode()),
      driveToCube1,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules()),
      new InstantCommand(() -> RobotContainer.bigStick.toggleDeploy()),
      new Shoot().withTimeout(0.75),
      new InstantCommand(() -> RobotContainer.bigStick.toggleDeploy()),
      driveOnCS,
      new AutoBalance(),
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules())
    );
  }

  public static CommandBase c3C0_CF_Bal() {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("c3C0_CF_Bal", 
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED / 1.75, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION));

    PPSwerveControllerCommand command = makeSwerveControllerCommand(pathGroup.get(0));
    PPSwerveControllerCommand command2 = makeSwerveControllerCommand(pathGroup.get(1));
    PPSwerveControllerCommand driveOnCS = makeSwerveControllerCommand(pathGroup.get(2));

    FollowPathWithEvents driveToCube1 = new FollowPathWithEvents(
      command,
      pathGroup.get(0).getMarkers(),
      RobotContainer.eventMap
    );

    FollowPathWithEvents driveToCube2 = new FollowPathWithEvents(
      command2,
      pathGroup.get(1).getMarkers(),
      RobotContainer.eventMap
    );
    
    return Commands.sequence(
      new InstantCommand(() -> RobotContainer.drivetrain.resetOdometry(getInitialPose(pathGroup.get(0)))),
      new InstantCommand(() -> RobotContainer.drivetrain.setAllIdleMode(true)),
      new InstantCommand(() -> RobotContainer.shooter.setCSMode()),
      new WaitCommand(0.75),
      new Shoot().withTimeout(0.5),
      new InstantCommand(() -> RobotContainer.intake.intakeToggle()),
      driveToCube1,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules()),
      new InstantCommand(() -> RobotContainer.bigStick.toggleDeploy()),
      new Shoot().withTimeout(0.5),
      new InstantCommand(() -> RobotContainer.bigStick.toggleDeploy()),
      driveToCube2,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules()),
      new InstantCommand(() -> RobotContainer.bigStick.toggleDeploy()),
      new Shoot().withTimeout(0.5),
      new InstantCommand(() -> RobotContainer.bigStick.toggleDeploy()),
      driveOnCS,
      new AutoBalance(),
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules())
    );
  }

  public static CommandBase c3C0_CF() {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("c3C0_CF", 
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION));

    PPSwerveControllerCommand command = makeSwerveControllerCommand(pathGroup.get(0));
    PPSwerveControllerCommand command2 = makeSwerveControllerCommand(pathGroup.get(1));

    FollowPathWithEvents driveToCube1 = new FollowPathWithEvents(
      command,
      pathGroup.get(0).getMarkers(),
      RobotContainer.eventMap
    );

    FollowPathWithEvents driveToCube2 = new FollowPathWithEvents(
      command2,
      pathGroup.get(1).getMarkers(),
      RobotContainer.eventMap
    );
    
    return Commands.sequence(
      new InstantCommand(() -> RobotContainer.drivetrain.resetOdometry(getInitialPose(pathGroup.get(0)))),
      new InstantCommand(() -> RobotContainer.drivetrain.setAllIdleMode(true)),
      new InstantCommand(() -> RobotContainer.shooter.setCSMode()),
      new WaitCommand(0.75),
      new Shoot().withTimeout(0.5),
      new InstantCommand(() -> RobotContainer.intake.intakeToggle()),
      driveToCube1,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules()),
      new InstantCommand(() -> RobotContainer.bigStick.toggleDeploy()),
      new Shoot().withTimeout(0.5),
      new InstantCommand(() -> RobotContainer.bigStick.toggleDeploy()),
      driveToCube2,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules()),
      new InstantCommand(() -> RobotContainer.bigStick.toggleDeploy()),
      new Shoot().withTimeout(0.5),
      new InstantCommand(() -> RobotContainer.bigStick.toggleDeploy())
    );
  }

  public static CommandBase c3C0_CN() {
    List<PathPlannerTrajectory> pathGroup;
    if(DriverStation.getAlliance().equals(Alliance.Blue)){
      pathGroup = PathPlanner.loadPathGroup("c3C0_CN_Blue", 
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED / 2.25, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED / 2.25, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION));
    }
    else{
        pathGroup = PathPlanner.loadPathGroup("c3C0_CN_Red", 
        new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
        new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED / 2.25, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
        new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
        new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
        new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED / 2.25, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
        new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION));
    }

    FollowPathWithEvents driveToBump = new FollowPathWithEvents(
      makeSwerveControllerCommand(pathGroup.get(0)),
      pathGroup.get(0).getMarkers(),
      RobotContainer.eventMap
    );

    FollowPathWithEvents driveOverBump1 = new FollowPathWithEvents(
      makeSwerveControllerCommand(pathGroup.get(1)),
      pathGroup.get(1).getMarkers(),
      RobotContainer.eventMap
    );

    FollowPathWithEvents driveToCube1 = new FollowPathWithEvents(
      makeSwerveControllerCommand(pathGroup.get(2)),
      pathGroup.get(2).getMarkers(),
      RobotContainer.eventMap
    );

    FollowPathWithEvents driveToCube2 = new FollowPathWithEvents(
      makeSwerveControllerCommand(pathGroup.get(3)),
      pathGroup.get(3).getMarkers(),
      RobotContainer.eventMap
    );

    FollowPathWithEvents driveOverBump2 = new FollowPathWithEvents(
      makeSwerveControllerCommand(pathGroup.get(4)),
      pathGroup.get(4).getMarkers(),
      RobotContainer.eventMap
    );

    FollowPathWithEvents driveToGrid = new FollowPathWithEvents(
      makeSwerveControllerCommand(pathGroup.get(5)),
      pathGroup.get(5).getMarkers(),
      RobotContainer.eventMap
    );
    
    return Commands.sequence(
      new InstantCommand(() -> RobotContainer.drivetrain.resetOdometry(getInitialPose(pathGroup.get(0)))),
      new InstantCommand(() -> RobotContainer.drivetrain.setAllIdleMode(true)),
      new InstantCommand(() -> RobotContainer.shooter.setHighMode()),
      new WaitCommand(0.75),
      new Shoot().withTimeout(0.5),
      new InstantCommand(() -> RobotContainer.shooter.setCSMode()),
      driveToBump,
      driveOverBump1,
      driveToCube1,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules()),
      new Shoot().withTimeout(0.5),
      new InstantCommand(() -> RobotContainer.shooter.setMidMode()),
      driveToCube2,
      driveOverBump2,
      driveToGrid,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules()),
      new Shoot().withTimeout(1)
    );
  }

  public static CommandBase c3C0_NC_Bal() {
    List<PathPlannerTrajectory> pathGroup = DriverStation.getAlliance().equals(Alliance.Red) ?
      PathPlanner.loadPathGroup("c3C0_NC_Bal_Red", 
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED / 1.75, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION)) :
      PathPlanner.loadPathGroup("c3C0_NC_Bal_Blue", 
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED / 1.75, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION));

    PPSwerveControllerCommand driveToCube1 = makeSwerveControllerCommand(pathGroup.get(0));
    PPSwerveControllerCommand driveToCube2 = makeSwerveControllerCommand(pathGroup.get(1));
    PPSwerveControllerCommand driveOnCS = makeSwerveControllerCommand(pathGroup.get(2));
    
    return Commands.sequence(
      new InstantCommand(() -> RobotContainer.drivetrain.resetOdometry(getInitialPose(pathGroup.get(0)))),
      new InstantCommand(() -> RobotContainer.drivetrain.setAllIdleMode(true)),
      new InstantCommand(() -> RobotContainer.shooter.setHighMode()),
      new WaitCommand(0.75),
      new Shoot().withTimeout(0.5),
      new InstantCommand(() -> RobotContainer.intake.intakeToggle()),
      new InstantCommand(() -> RobotContainer.shooter.setCSMode()),
      driveToCube1,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules()),
      new Shoot().withTimeout(0.5),
      driveToCube2,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules()),
      new Shoot().withTimeout(0.5),
      driveOnCS,
      new AutoBalance(),
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules())
    );
  }

  public static CommandBase c3C0_NC() {
    List<PathPlannerTrajectory> pathGroup = DriverStation.getAlliance().equals(Alliance.Red) ?
      PathPlanner.loadPathGroup("c3C0_NC_Red", 
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION)) :
      PathPlanner.loadPathGroup("c3C0_NC_Blue", 
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION));

    PPSwerveControllerCommand driveToCube1 = makeSwerveControllerCommand(pathGroup.get(0));
    PPSwerveControllerCommand driveToCube2 = makeSwerveControllerCommand(pathGroup.get(1));
    PPSwerveControllerCommand driveToLoadZone = makeSwerveControllerCommand(pathGroup.get(2));
    
    return Commands.sequence(
      new InstantCommand(() -> RobotContainer.drivetrain.resetOdometry(getInitialPose(pathGroup.get(0)))),
      new InstantCommand(() -> RobotContainer.drivetrain.setAllIdleMode(true)),
      new InstantCommand(() -> RobotContainer.shooter.setHighMode()),
      new WaitCommand(0.75),
      new Shoot().withTimeout(0.5),
      new InstantCommand(() -> RobotContainer.intake.intakeToggle()),
      new InstantCommand(() -> RobotContainer.shooter.setMidMode()),
      driveToCube1,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules()),
      new Shoot().withTimeout(0.5),
      new InstantCommand(() -> RobotContainer.shooter.setCSMode()),
      driveToCube2,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules()),
      new Shoot().withTimeout(0.5),
      new InstantCommand(() -> RobotContainer.intake.intakeToggle()),
      driveToLoadZone,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules())
    );
  }

  public static CommandBase c4C0_NC() {
    List<PathPlannerTrajectory> pathGroup = DriverStation.getAlliance().equals(Alliance.Red) ?
      PathPlanner.loadPathGroup("c4C0_NC_Red", 
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION)) :
      PathPlanner.loadPathGroup("c4C0_NC_Blue", 
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION));

    PPSwerveControllerCommand driveToCube1 = makeSwerveControllerCommand(pathGroup.get(0));
    PPSwerveControllerCommand driveToCube2 = makeSwerveControllerCommand(pathGroup.get(1));
    PPSwerveControllerCommand driveToCube3 = makeSwerveControllerCommand(pathGroup.get(2));
    PPSwerveControllerCommand drivetoLoadZone = makeSwerveControllerCommand(pathGroup.get(3));

    return Commands.sequence(
      new InstantCommand(() -> RobotContainer.drivetrain.resetOdometry(getInitialPose(pathGroup.get(0)))),
      new InstantCommand(() -> RobotContainer.drivetrain.setAllIdleMode(true)),
      new InstantCommand(() -> RobotContainer.shooter.setHighMode()),
      new WaitCommand(0.75),
      new Shoot().withTimeout(0.5),
      new InstantCommand(() -> RobotContainer.intake.intakeToggle()),
      new InstantCommand(() -> RobotContainer.shooter.setCSMode()),
      driveToCube1,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules()),
      new Shoot().withTimeout(0.5),
      driveToCube2,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules()),
      new Shoot().withTimeout(0.5),
      driveToCube3,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules()),
      new Shoot().withTimeout(0.5),
      new InstantCommand(() -> RobotContainer.intake.intakeToggle()),
      drivetoLoadZone,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules())
    );
  }

  public static CommandBase c4C0_NC_Bal() {
    List<PathPlannerTrajectory> pathGroup = DriverStation.getAlliance().equals(Alliance.Red) ?
      PathPlanner.loadPathGroup("c4C0_NC_Bal_Red", 
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED / 1.75, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION)) :
      PathPlanner.loadPathGroup("c4C0_NC_Bal_Blue", 
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED / 1.75, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION));

    PPSwerveControllerCommand command = makeSwerveControllerCommand(pathGroup.get(0));
    PPSwerveControllerCommand command2 = makeSwerveControllerCommand(pathGroup.get(1));

    FollowPathWithEvents driveToCubes = new FollowPathWithEvents(
      command,
      pathGroup.get(0).getMarkers(),
      RobotContainer.eventMap
    );

    FollowPathWithEvents driveOnCS = new FollowPathWithEvents(
      command2,
      pathGroup.get(1).getMarkers(),
      RobotContainer.eventMap
    );

    return Commands.sequence(
      new InstantCommand(() -> RobotContainer.drivetrain.resetOdometry(getInitialPose(pathGroup.get(0)))),
      new InstantCommand(() -> RobotContainer.drivetrain.setAllIdleMode(true)),
      new InstantCommand(() -> RobotContainer.shooter.setHighMode()),
      new WaitCommand(0.75),
      new Shoot().withTimeout(0.5),
      new InstantCommand(() -> RobotContainer.shooter.setCSMode()),
      driveToCubes,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules()),
      driveOnCS,
      new AutoBalance(),
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules())
    );
  }

  public static CommandBase c4C0_CF() {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("c4C0_CF", 
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION));

    PPSwerveControllerCommand driveToCube1 = makeSwerveControllerCommand(pathGroup.get(0));
    PPSwerveControllerCommand driveToCube2 = makeSwerveControllerCommand(pathGroup.get(1));
    PPSwerveControllerCommand driveToCube3 = makeSwerveControllerCommand(pathGroup.get(2));

    return Commands.sequence(
      new InstantCommand(() -> RobotContainer.drivetrain.resetOdometry(getInitialPose(pathGroup.get(0)))),
      new InstantCommand(() -> RobotContainer.drivetrain.setAllIdleMode(true)),
      new InstantCommand(() -> RobotContainer.shooter.setCSMode()),
      new WaitCommand(0.75),
      new Shoot().withTimeout(0.5),
      new InstantCommand(() -> RobotContainer.intake.intakeToggle()),
      driveToCube1,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules()),
      new Shoot().withTimeout(0.5),
      driveToCube2,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules()),
      new Shoot().withTimeout(0.5),
      driveToCube3,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules()),
      new Shoot().withTimeout(0.5),
      new InstantCommand(() -> RobotContainer.intake.intakeToggle())
    );
  }

  public static CommandBase c4C0_CF_Bal() {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("c4C0_CF_Bal", 
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION),
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED / 1.75, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION));

    PPSwerveControllerCommand command = makeSwerveControllerCommand(pathGroup.get(0));
    PPSwerveControllerCommand command2 = makeSwerveControllerCommand(pathGroup.get(1));

    FollowPathWithEvents driveToCubes = new FollowPathWithEvents(
      command,
      pathGroup.get(0).getMarkers(),
      RobotContainer.eventMap
    );

    FollowPathWithEvents driveOnCS = new FollowPathWithEvents(
      command2,
      pathGroup.get(1).getMarkers(),
      RobotContainer.eventMap
    );

    return Commands.sequence(
      new InstantCommand(() -> RobotContainer.drivetrain.resetOdometry(getInitialPose(pathGroup.get(0)))),
      new InstantCommand(() -> RobotContainer.drivetrain.setAllIdleMode(true)),
      new InstantCommand(() -> RobotContainer.shooter.setCSMode()),
      new WaitCommand(0.75),
      new Shoot().withTimeout(0.5),
      driveToCubes,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules()),
      driveOnCS,
      new AutoBalance(),
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules())
    );
  }

  public static CommandBase c5C0_NC() {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("c5C0_NC", 
      new PathConstraints(SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION));

    PPSwerveControllerCommand command = makeSwerveControllerCommand(pathGroup.get(0));

    FollowPathWithEvents driveToCubes = new FollowPathWithEvents(
      command,
      pathGroup.get(0).getMarkers(),
      RobotContainer.eventMap
    );
    
    return Commands.sequence(
      new InstantCommand(() -> RobotContainer.drivetrain.resetOdometry(getInitialPose(pathGroup.get(0)))),
      new InstantCommand(() -> RobotContainer.drivetrain.setAllIdleMode(true)),
      new InstantCommand(() -> RobotContainer.shooter.setHighMode()),
      new WaitCommand(0.75),
      new Shoot().withTimeout(0.5),
      new InstantCommand(() -> RobotContainer.shooter.setCSMode()),
      driveToCubes,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules()),
      new Shoot().withTimeout(0.5),
      new InstantCommand(() -> RobotContainer.bigStick.toggleDeploy())
    );
  }

  public static CommandBase driveBack() {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath("DriveBack", SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION);

    PPSwerveControllerCommand driveBack = makeSwerveControllerCommand(trajectory);

    return new SequentialCommandGroup(
      new InstantCommand(() -> RobotContainer.drivetrain.resetOdometry(getInitialPose(trajectory))),
      new InstantCommand(() -> RobotContainer.drivetrain.setAllIdleMode(true)),
      new InstantCommand(() -> RobotContainer.shooter.setHighMode()),
      new WaitCommand(0.75),
      new Shoot().withTimeout(0.5),
      driveBack,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules())
    );
  }

  public static CommandBase testAuto(){
    return new SequentialCommandGroup(
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules())
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
