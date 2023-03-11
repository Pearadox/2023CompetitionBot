// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.drivers.EForwardableConnections;
import frc.lib.drivers.Launchpad;
import frc.lib.drivers.LaunchpadButton;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.ArmHold;
import frc.robot.commands.Autos;
import frc.robot.commands.BigStickHold;
import frc.robot.commands.IntakeHold;
import frc.robot.commands.IntakeRollersHold;
import frc.robot.commands.Outtake;
import frc.robot.commands.Shoot;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BigStick;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transport;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final Drivetrain drivetrain = Drivetrain.getInstance();
  public static final Intake intake = Intake.getInstance();
  public static final IntakeRollers intakeRollers = IntakeRollers.getInstance();
  public static final Arm arm = Arm.getInstance();
  public static final Shooter shooter = Shooter.getInstance();
  public static final BigStick bigStick = BigStick.getInstance();
  public static final Transport transport = Transport.getInstance();

  public static final PowerDistribution pdh = new PowerDistribution(Constants.PDH_ID, ModuleType.kRev);

  private SendableChooser<String> autoChooser = new SendableChooser<>();

  public static final XboxController driverController = new XboxController(IOConstants.DRIVER_CONTROLLER_PORT);
  private final JoystickButton resetHeading_Start = new JoystickButton(driverController, XboxController.Button.kStart.value);
  private final JoystickButton toggleIntake_LB = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
  private final JoystickButton shoot_RB = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
  private final JoystickButton armScore_B = new JoystickButton(driverController, XboxController.Button.kB.value);
  private final JoystickButton outtake_X = new JoystickButton(driverController, XboxController.Button.kX.value);
  private final JoystickButton subsDriveMode_Y = new JoystickButton(driverController, XboxController.Button.kY.value);
  private final JoystickButton gridDriveMode_A = new JoystickButton(driverController, XboxController.Button.kA.value);
  
  public static final Launchpad opController = new Launchpad();
  // private final LaunchpadButton[][] gridButtons = new LaunchpadButton[3][9];
  private final LaunchpadButton armHigh_1_0 = new LaunchpadButton(opController, 1, 0);
  private final LaunchpadButton armMid_2_0 = new LaunchpadButton(opController, 2, 0);
  private final LaunchpadButton armLow_3_0 = new LaunchpadButton(opController, 3, 0);
  private final LaunchpadButton armZero_4_0 = new LaunchpadButton(opController, 4, 0);
  private final LaunchpadButton armSubs_2_1 = new LaunchpadButton(opController, 2, 1);

  private final LaunchpadButton armAdjustUp_1_3 = new LaunchpadButton(opController, 1, 3);
  private final LaunchpadButton armAdjustDown_2_3 = new LaunchpadButton(opController, 2, 3);

  private final LaunchpadButton shooterHigh_1_7 = new LaunchpadButton(opController, 1, 7);
  private final LaunchpadButton shooterMid_2_7 = new LaunchpadButton(opController, 2, 7);
  private final LaunchpadButton shooterCS_3_7 = new LaunchpadButton(opController, 3, 7);

  private final LaunchpadButton toggleBigStick_2_5 = new LaunchpadButton(opController, 2, 5);

  private final LaunchpadButton feederOut_4_2 = new LaunchpadButton(opController, 4, 2);
  private final LaunchpadButton feederIn_4_3 = new LaunchpadButton(opController, 4, 3);

  private final LaunchpadButton intakeToggle_1_5 = new LaunchpadButton(opController, 1, 5);

  public static final XboxController backupOpController = new XboxController(IOConstants.OP_CONTROLLER_PORT);
  private final JoystickButton armUp_Y = new JoystickButton(backupOpController, XboxController.Button.kY.value);
  private final JoystickButton armDown_A = new JoystickButton(backupOpController, XboxController.Button.kA.value);
  private final JoystickButton armSubs_X = new JoystickButton(backupOpController, XboxController.Button.kX.value);
  private final JoystickButton toggleBigStick_RB = new JoystickButton(backupOpController, XboxController.Button.kRightBumper.value);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    portForwarding();
    configureBindings();
    drivetrain.setDefaultCommand(new SwerveDrive());
    intake.setDefaultCommand(new IntakeHold());
    intakeRollers.setDefaultCommand(new IntakeRollersHold());
    arm.setDefaultCommand(new ArmHold());
    bigStick.setDefaultCommand(new BigStickHold());

    SmartDashboard.putData("Auton Chooser", autoChooser);
    autoChooser.setDefaultOption("1CubeM_Bal", "1CubeM_Bal");
    autoChooser.addOption("2CubeNC", "2CubeNC");
    autoChooser.addOption("2CubeNC_Bal", "2CubeNC_Bal");
    autoChooser.addOption("3CubeNC_Bal", "3CubeNC_Bal");
    autoChooser.addOption("4CubeNC", "4CubeNC");
    // autoChooser.addOption("1Cone1CubeNC_Bal", "1Cone1CubeNC_Bal");
    // autoChooser.addOption("1Cone1CubeC_Bal", "1Cone1CubeC_Bal");
    autoChooser.addOption("Nothing", "TestAuto");
    autoChooser.addOption("DriveBack", "DriveBack");
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    resetHeading_Start.onTrue(new InstantCommand(drivetrain::zeroHeading, drivetrain));
    toggleIntake_LB.onTrue(new InstantCommand(intake::intakeToggle, intake));
    shoot_RB.whileTrue(new Shoot(0.75)).onFalse(new InstantCommand(() -> transport.feederStop())
      .andThen(new InstantCommand(() -> shooter.shooterOff())));
    armScore_B.whileTrue(new RunCommand(() -> arm.intakeOut())).onFalse(new InstantCommand(() -> arm.intakeIn()));
    outtake_X.whileTrue(new Outtake()).onFalse(new InstantCommand(() -> transport.feederStop()));
    gridDriveMode_A.whileTrue(new RunCommand(() -> drivetrain.setGridMode())).onFalse(new InstantCommand(() -> drivetrain.setNormalMode()));
    subsDriveMode_Y.whileTrue(new RunCommand(() -> drivetrain.setSubsMode())).onFalse(new InstantCommand(() -> drivetrain.setNormalMode()));


    armHigh_1_0.onTrue(new InstantCommand(() -> arm.setHighMode()));
    armMid_2_0.onTrue(new InstantCommand(() -> arm.setMidMode()));
    armLow_3_0.onTrue(new InstantCommand(() -> arm.setLowMode()));
    armZero_4_0.onTrue(new InstantCommand(() -> arm.setZeroMode()));
    armSubs_2_1.onTrue(new InstantCommand(() -> arm.setSubsMode()));

    armAdjustUp_1_3.onTrue(new InstantCommand(() -> arm.armAdjustUp()));
    armAdjustDown_2_3.onTrue(new InstantCommand(() -> arm.armAdjustDown()));

    shooterHigh_1_7.onTrue(new InstantCommand(() -> shooter.setHighMode()));
    shooterMid_2_7.onTrue(new InstantCommand(() -> shooter.setMidMode()));
    shooterCS_3_7.onTrue(new InstantCommand(() -> shooter.setCSMode()));

    toggleBigStick_2_5.onTrue(new InstantCommand(() -> bigStick.toggleDeploy()));

    feederOut_4_2.whileTrue(new RunCommand(() -> transport.feederOut(-0.1))).onFalse(new InstantCommand(() -> transport.feederStop()));
    feederIn_4_3.whileTrue(new RunCommand(() -> transport.feederHold())).onFalse(new InstantCommand(() -> transport.feederStop()));

    intakeToggle_1_5.onTrue(new InstantCommand(intake::intakeToggle, intake));

    armUp_Y.onTrue(new InstantCommand(() -> arm.armUp()));
    armDown_A.onTrue(new InstantCommand(() -> arm.armDown()));
    armSubs_X.onTrue(new InstantCommand(() -> arm.setSubsMode()));
    toggleBigStick_RB.onTrue(new InstantCommand(() -> bigStick.toggleDeploy()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    if(autoChooser.getSelected().equals("1CubeM_Bal")){
      drivetrain.resetAllEncoders();
      drivetrain.setHeading(0);
      return Autos.c1C0_M_Bal();
    }
    else if(autoChooser.getSelected().equals("2CubeNC")){
      drivetrain.resetAllEncoders();
      drivetrain.setHeading(0);
      return Autos.c2C0_NC();
    }
    else if(autoChooser.getSelected().equals("2CubeNC_Bal")){
      drivetrain.resetAllEncoders();
      drivetrain.setHeading(0);
      return Autos.c2C0_NC_Bal();
    }
    else if(autoChooser.getSelected().equals("3CubeNC_Bal")){
      drivetrain.resetAllEncoders();
      drivetrain.setHeading(0);
      return Autos.c3C0_NC_Bal();
    }
    else if(autoChooser.getSelected().equals("4CubeNC")){
      drivetrain.resetAllEncoders();
      drivetrain.setHeading(0);
      return Autos.c4C0_NC();
    }
    // else if(autoChooser.getSelected().equals("1Cone1CubeNC_Bal")){
    //   return Autos.c1C1_NC_Bal();
    // }
    // else if(autoChooser.getSelected().equals("1Cone1CubeC_Bal")){
    //   return Autos.c1C1_C_Bal();
    // }
    else if(autoChooser.getSelected().equals("TestAuto")){
      drivetrain.resetAllEncoders();
      drivetrain.setHeading(0);
      return Autos.testAuto();
    }
    else if(autoChooser.getSelected().equals("DriveBack")){
      drivetrain.resetAllEncoders();
      drivetrain.setHeading(0);
      return Autos.driveBack();
    }
    else{
      drivetrain.resetAllEncoders();
      drivetrain.setHeading(0);
      return Autos.driveBack();
    }
  }

  private void portForwarding(){
    EForwardableConnections.addPortForwarding(EForwardableConnections.LIMELIGHT_ARM_CAMERA_FEED);
    EForwardableConnections.addPortForwarding(EForwardableConnections.LIMELIGHT_ARM_WEB_VIEW);
  }

  // public void loadGridButtons(){
  //   for(int r = 0; r < 3; r++){
  //     for(int c = 0; c < 9; c++){
  //       gridButtons[r][c] = new LaunchpadButton(opController, r, c);
  //     }
  //   }
  // }
}
