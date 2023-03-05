// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.drivers.Launchpad;
import frc.lib.drivers.LaunchpadButton;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.ArmHold;
import frc.robot.commands.Autos;
import frc.robot.commands.BigStickHold;
import frc.robot.commands.IntakeHold;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShooterHold;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BigStick;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
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
  public static final Arm arm = Arm.getInstance();
  public static final Shooter shooter = Shooter.getInstance();
  public static final BigStick bigStick = BigStick.getInstance();

  public static final PowerDistribution pdh = new PowerDistribution(Constants.PDH_ID, ModuleType.kRev);

  private SendableChooser<String> autoChooser = new SendableChooser<>();

  public static final XboxController driverController = new XboxController(IOConstants.DRIVER_CONTROLLER_PORT);
  private final JoystickButton resetHeading_Start = new JoystickButton(driverController, XboxController.Button.kStart.value);
  private final JoystickButton toggleIntake_LB = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
  private final JoystickButton shoot_RB = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
  private final JoystickButton armScore_B = new JoystickButton(driverController, XboxController.Button.kB.value);

  public static final Launchpad opController = new Launchpad();
  // private final LaunchpadButton[][] gridButtons = new LaunchpadButton[3][9];
  private final LaunchpadButton armHigh_1_0 = new LaunchpadButton(opController, 1, 0);
  private final LaunchpadButton armMid_2_0 = new LaunchpadButton(opController, 2, 0);
  private final LaunchpadButton armLow_3_0 = new LaunchpadButton(opController, 3, 0);
  private final LaunchpadButton armZero_4_0 = new LaunchpadButton(opController, 4, 0);
  private final LaunchpadButton armSubs_2_1 = new LaunchpadButton(opController, 2, 1);

  private final LaunchpadButton shooterHigh_1_7 = new LaunchpadButton(opController, 1, 7);
  private final LaunchpadButton shooterMid_2_7 = new LaunchpadButton(opController, 2, 7);
  private final LaunchpadButton shooterCS_3_7 = new LaunchpadButton(opController, 3, 7);

  private final LaunchpadButton toggleBigStick_2_5 = new LaunchpadButton(opController, 2, 5);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    drivetrain.setDefaultCommand(new SwerveDrive());
    intake.setDefaultCommand(new IntakeHold());
    arm.setDefaultCommand(new ArmHold());
    shooter.setDefaultCommand(new ShooterHold());
    bigStick.setDefaultCommand(new BigStickHold());

    SmartDashboard.putData("Auton Chooser", autoChooser);
    autoChooser.setDefaultOption("1CubeM_Bal", "1CubeM_Bal");
    autoChooser.addOption("2CubeNC_Bal", "2CubeNC_Bal");
    autoChooser.addOption("1Cone1CubeNC_Bal", "1Cone1CubeNC_Bal");
    autoChooser.addOption("1Cone1CubeC_Bal", "1Cone1CubeC_Bal");
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
    shoot_RB.whileTrue(new Shoot());
    armScore_B.whileTrue(new RunCommand(() -> arm.intakeOut()));

    armHigh_1_0.onTrue(new InstantCommand(() -> arm.setHighMode()));
    armMid_2_0.onTrue(new InstantCommand(() -> arm.setMidMode()));
    armLow_3_0.onTrue(new InstantCommand(() -> arm.setLowMode()));
    armZero_4_0.onTrue(new InstantCommand(() -> arm.setZeroMode()));
    armSubs_2_1.onTrue(new InstantCommand(() -> arm.setSubsMode()));

    shooterHigh_1_7.onTrue(new InstantCommand(() -> shooter.setHighMode()));
    shooterMid_2_7.onTrue(new InstantCommand(() -> shooter.setMidMode()));
    shooterCS_3_7.onTrue(new InstantCommand(() -> shooter.setCSMode()));

    toggleBigStick_2_5.onTrue(new InstantCommand(() -> bigStick.toggleDeploy()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    if(autoChooser.getSelected().equals("1CubeM_Bal")){
      return Autos.c1C0_M_Bal();
    }
    else if(autoChooser.getSelected().equals("2CubeNC_Bal")){
      return Autos.c2C0_NC_Bal();
    }
    else if(autoChooser.getSelected().equals("1Cone1CubeNC_Bal")){
      return Autos.c1C1_NC_Bal();
    }
    else if(autoChooser.getSelected().equals("1Cone1CubeC_Bal")){
      return Autos.c1C1_C_Bal();
    }
    else{
      return Autos.c1C0_M_Bal();
    }
  }

  // public void loadGridButtons(){
  //   for(int r = 0; r < 3; r++){
  //     for(int c = 0; c < 9; c++){
  //       gridButtons[r][c] = new LaunchpadButton(opController, r, c);
  //     }
  //   }
  // }
}
