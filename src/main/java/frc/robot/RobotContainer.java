// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.IntakeSetFourBar;
import frc.robot.commands.IntakeSetRollers;
import frc.robot.commands.auto.FourBall;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeFourBar;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem robotIntake = new IntakeSubsystem();
  private final IntakeFourBar fourBar = new IntakeFourBar();
  // public static final ShootSubsystem shootSubsystem = new ShootSubsystem();
  // private final SnekSystem snekSystem = new SnekSystem();

  public final XboxController controller = new XboxController(Constants.zero);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    driveSubsystem.setDefaultCommand(
        new RunCommand(
            () -> {
              driveSubsystem.GTADrive(
                  controller.getLeftTriggerAxis(),
                  controller.getRightTriggerAxis(),
                  controller.getLeftX());
            },
            driveSubsystem));

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
    // new JoystickButton(controller, XboxController.Button.kA.value)
    // .whenPressed(
    // () -> {
    // shootSubsystem.setTargetRPM(Constants.ShooterConstants.RPM);
    // });

    // new JoystickButton(controller, XboxController.Button.kB.value)
    // .whenPressed(
    // () -> {
    // shootSubsystem.setTargetRPM(Constants.zero);
    // });

    // new JoystickButton(controller, XboxController.Button.kY.value)
    // .whenActive(new IntakeSetRollers(robotIntake,
    // Constants.IntakeConstants.speed))
    // .whenInactive(new IntakeSetRollers(robotIntake, Constants.zero));

    new JoystickButton(controller, XboxController.Button.kB.value)
        .whenActive(new IntakeSetFourBar(fourBar, Constants.IntakeConstants.extensionPoint))
        .whenInactive(new IntakeSetFourBar(fourBar, 0));

    new JoystickButton(controller, XboxController.Button.kA.value)
        .whenActive(new IntakeSetRollers(robotIntake, Constants.IntakeConstants.typicalRollerRPM))
        .whenInactive(new IntakeSetRollers(robotIntake, 0));

    new JoystickButton(controller, XboxController.Button.kY.value)
        .whenActive(
            () -> {
              fourBar.zero();
            });
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return new FourBall(driveSubsystem)
        .andThen(() -> driveSubsystem.tankDriveVolts(Constants.zero, Constants.zero));
  }
}
