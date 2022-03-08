// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.IntakeSetFourBar;
import frc.robot.commands.IntakeSetRollers;
import frc.robot.commands.SetShooterRPM;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeFourBar;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.SnekSystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final DriveSubsystem driveSubsystem = new DriveSubsystem();
  public static final IntakeSubsystem robotIntake = new IntakeSubsystem();
  public static final IntakeFourBar fourBar = new IntakeFourBar();
  public static final ShootSubsystem shootSubsystem = new ShootSubsystem();
  public static final SnekSystem snekSystem = new SnekSystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();

  public final XboxController driver = new XboxController(Constants.zero);
  public final XboxController operator = new XboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    driveSubsystem.setDefaultCommand(
        new RunCommand(
            () -> {
              driveSubsystem.GTADrive(
                  driver.getLeftTriggerAxis(), driver.getRightTriggerAxis(), driver.getLeftX());
            },
            driveSubsystem));

    climber.setDefaultCommand(
        new RunCommand(
            () -> {
              climber.setTelescopeSpeed(-MathUtil.applyDeadband(operator.getRightY(), 0.1));
            },
            climber));

    snekSystem.setDefaultCommand(
        new RunCommand(
            () -> {
              snekSystem.loadSnek();
            },
            snekSystem));

    // fourBar.setDefaultCommand(
    // new RunCommand(
    // () -> {
    // fourBar.setFourBarMotor(controller.getRightX());
    // },
    // fourBar));

    // snekSystem.setDefaultCommand(
    // new RunCommand(
    // () -> {
    // snekSystem.loadSnek();
    // },
    // snekSystem));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // new JoystickButton(operator, XboxController.Button.kA.value)
    // // .whileActiveContinuous(new ForceSnek(snekSystem));
    // .whenActive(
    // new InstantCommand(
    // () -> {
    // snekSystem.setLowerSnekSpeed(0.5);
    // snekSystem.setUpperSnekSpeed(0.5);
    // },
    // snekSystem))
    // .whenInactive(
    // new InstantCommand(
    // () -> {
    // snekSystem.setLowerSnekSpeed(0);
    // snekSystem.setUpperSnekSpeed(0);
    // },
    // snekSystem));

    // snekSystem.setDefaultCommand(
    // new RunCommand(
    // () -> {
    // snekSystem.setLowerSnekSpeed(operator.getLeftTriggerAxis());
    // snekSystem.setUpperSnekSpeed(operator.getRightTriggerAxis());
    // },
    // snekSystem));

    new JoystickButton(operator, XboxController.Button.kLeftBumper.value)
        .whileActiveOnce(
            new SequentialCommandGroup(
                new RunCommand(
                        () -> {
                          snekSystem.setUpperSnekSpeed(-0.4);
                          snekSystem.setLowerSnekSpeed(-0.1);
                        },
                        snekSystem)
                    .withTimeout(0.25),
                new SetShooterRPM(
                    shootSubsystem,
                    Constants.ShooterConstants.typicalShotSpeed.get(),
                    Constants.ShooterConstants.waitUntilAtSpeed)))
        .whenInactive(
            new SetShooterRPM(
                shootSubsystem, Constants.zero, Constants.ShooterConstants.waitUntilAtSpeed));

    // new JoystickButton(driver, XboxController.Button.kB.value)
    // .whenPressed(
    // () -> {
    // shootSubsystem.setTargetRPM(Constants.zero);
    // });

    new JoystickButton(driver, XboxController.Button.kY.value)
        .whenPressed(
            new ParallelCommandGroup(
                new IntakeSetRollers(robotIntake, Constants.IntakeConstants.typicalRollerRPM),
                new IntakeSetFourBar(fourBar, Constants.IntakeConstants.extensionPoint)))
        .whenReleased(
            new ParallelCommandGroup(
                new IntakeSetRollers(robotIntake, Constants.zero),
                new IntakeSetFourBar(fourBar, 0)));

    // climbSubsystem code - should use X, with manual input from the vertical axis
    // of the second
    // stick
    new JoystickButton(operator, XboxController.Button.kA.value)
        .whileHeld(
            new InstantCommand(
                () -> {
                  snekSystem.setLowerSnekSpeed(1);
                  snekSystem.setUpperSnekSpeed(1);
                },
                snekSystem))
        .whenReleased(
            new InstantCommand(
                () -> {
                  snekSystem.setLowerSnekSpeed(0);
                  snekSystem.setUpperSnekSpeed(0);
                },
                snekSystem));

    new JoystickButton(operator, XboxController.Button.kB.value)
        .whileHeld(
            new InstantCommand(
                () -> {
                  snekSystem.setLowerSnekSpeed(-1);
                  snekSystem.setUpperSnekSpeed(-1);
                },
                snekSystem));

    // new JoystickButton(controller, XboxController.Button.kB.value)
    // .whenPressed(
    // () -> {
    // shootSubsystem.setTargetRPM(Constants.zero);
    // });

    // new JoystickButton(controller, XboxController.Button.kY.value)
    // .whenActive(new IntakeSetRollers(robotIntake,
    // Constants.IntakeConstants.speed))
    // .whenInactive(new IntakeSetRollers(robotIntake, Constants.zero));

    // new JoystickButton(controller, XboxController.Button.kB.value)
    // .whileActiveOnce(new IntakeSetFourBar(fourBar,
    // Constants.IntakeConstants.extensionPoint))
    // .whenInactive(new IntakeSetFourBar(fourBar, 0));

    // new JoystickButton(controller, XboxController.Button.kA.value)
    // .whileActiveOnce(
    // new IntakeSetRollers(robotIntake,
    // Constants.IntakeConstants.typicalRollerRPM))
    // .whenInactive(new IntakeSetRollers(robotIntake, 0));
  }
}
