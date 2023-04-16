// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    setUseTiming(isReal());
    Logger.getInstance().recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    if (isReal()) {
        Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
        Logger.getInstance().addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else {
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.getInstance().setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.getInstance().addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    // Logger.getInstance().disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
    Logger.getInstance().start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.


    m_robotContainer = new RobotContainer();
    RobotContainer.pdh.setSwitchableChannel(false);
    RobotContainer.ledStrip.setDefaultMode();
    PortForwarder.add(8888, "limelight.local", 5800);
    PortForwarder.add(8889, "limelight.local", 5801);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putBoolean("Auto Is Valid?", m_robotContainer.isAutoValid());

    Logger.getInstance().recordOutput("Autonomous/Valid", m_robotContainer.isAutoValid());
    Logger.getInstance().recordOutput("Autonomous/Starting Side", m_robotContainer.getAutoStartingSide());
    Logger.getInstance().recordOutput("Autonomous/Game Pieces", m_robotContainer.getAutoGamePieces());
    Logger.getInstance().recordOutput("Autonomous/Balance", m_robotContainer.getAutoBalance());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    RobotContainer.drivetrain.setAllIdleMode(true);
    RobotContainer.pdh.setSwitchableChannel(false);
    if(!RobotContainer.ledStrip.isRainbowMode()){
      RobotContainer.ledStrip.setDefaultMode();
    }
  }

  @Override
  public void disabledPeriodic() {
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    RobotContainer.pdh.setSwitchableChannel(true);
    RobotContainer.ledStrip.setCubeMode();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    RobotContainer.drivetrain.setAllIdleMode(true);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    RobotContainer.drivetrain.resetAllEncoders();
    RobotContainer.drivetrain.setNormalMode();
    RobotContainer.pdh.setSwitchableChannel(true);
    RobotContainer.ledStrip.setCubeMode();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // if(RobotContainer.arm.isDeployed() ? 
    //     (RobotContainer.intake.isDeployed() || RobotContainer.bigStick.isDeployed()) : 
    //     (RobotContainer.intake.isDeployed() && RobotContainer.bigStick.isDeployed())){
    //   RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 1);
    // }
    // else{
    //   RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 0);
    // }

    // RobotContainer.poseEstimator.periodic();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
