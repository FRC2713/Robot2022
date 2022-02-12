// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.DriveSubsystem;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final DriveSubsystem driveSubsystem = new DriveSubsystem();
  //  private final IntakeSubsystem robotIntake = new IntakeSubsystem();
  //  private final IntakeFourBar intakeFourBar = new IntakeFourBar();
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

    driveSubsystem.resetEncoders();

    // snekSystem.setDefaultCommand(
    //     new RunCommand(
    //         () -> {
    //           snekSystem.loadSnek();
    //         },
    //         snekSystem));

    //   try {
    //     Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    //     trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    //  } catch (IOException ex) {
    //     DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON,
    // ex.getStackTrace());
    //  }
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

    //   new JoystickButton(controller, XboxController.Button.kY.value)
    //        .whenPressed(new DeployIntake(robotIntake, intakeFourBar));
    // //   .whenReleased(new StashIntake(robotIntake, intakeFourBar));

    //   new JoystickButton(controller, XboxController.Button.kX.value)
    //       .whenPressed(new StashIntake(robotIntake, intakeFourBar));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                Constants.AutoConstants.ksVolts,
                Constants.AutoConstants.ksVoltSecondsPerMeter,
                Constants.AutoConstants.kaVoltSecondsSquaredPerMeter),
            Constants.AutoConstants.kinematics,
            6);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(Constants.AutoConstants.maxSpeed, Constants.AutoConstants.maxAccel)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.AutoConstants.kinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, 0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);

    RamseteCommand ramsete =
        new RamseteCommand(
            exampleTrajectory,
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

    // driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

    return ramsete.andThen(() -> driveSubsystem.tankDriveVolts(Constants.zero, Constants.zero));
  }
}
