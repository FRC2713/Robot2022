// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.StashIntake;
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
   private final IntakeFourBar intakeFourBar = new IntakeFourBar();
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

    // snekSystem.setDefaultCommand(
    //     new RunCommand(
    //         () -> {
    //           snekSystem.loadSnek();
    //         },
    //         snekSystem));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // new JoystickButton(controller, XboxController.Button.kA.value)
    //     .whenPressed(
    //         () -> {
    //           shootSubsystem.setTargetRPM(Constants.ShooterConstants.RPM);
    //         });

    // new JoystickButton(controller, XboxController.Button.kB.value)
    //     .whenPressed(
    //         () -> {
    //           shootSubsystem.setTargetRPM(Constants.zero);
    //         });

    new JoystickButton(controller, XboxController.Button.kY.value)
         .whenPressed(new DeployIntake(robotIntake, intakeFourBar));
  //   .whenReleased(new StashIntake(robotIntake, intakeFourBar));


    new JoystickButton(controller, XboxController.Button.kX.value)
        .whenPressed(new StashIntake(robotIntake, intakeFourBar));
  }

  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    PathPlannerTrajectory autoTrajectory =
        PathPlanner.loadPath(
            "test", Constants.AutoConstants.maxSpeed, Constants.AutoConstants.maxAccel);

    RamseteCommand ramsete =
        new RamseteCommand(
            autoTrajectory,
            driveSubsystem::getPose,
            new RamseteController(
                Constants.AutoConstants.RamseteB, Constants.AutoConstants.RamseteZeta),
            new SimpleMotorFeedforward(
                Constants.AutoConstants.ksVolts,
                Constants.AutoConstants.ksVoltSecondsPerMeter,
                Constants.AutoConstants.kaVoltSecondsSquaredPerMeter),
            Constants.AutoConstants.kinematics,
            driveSubsystem::getWheelSpeeds,
            new PIDController(Constants.AutoConstants.kPDriveVel, Constants.zero, Constants.zero),
            new PIDController(Constants.AutoConstants.kPDriveVel, Constants.zero, Constants.zero),
            driveSubsystem::tankDriveVolts,
            driveSubsystem);

    driveSubsystem.resetOdometry(autoTrajectory.getInitialPose());

    return ramsete.andThen(() -> driveSubsystem.tankDriveVolts(Constants.zero, Constants.zero));
  }
}
