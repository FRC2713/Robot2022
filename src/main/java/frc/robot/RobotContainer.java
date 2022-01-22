// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

<<<<<<< Updated upstream
=======
import com.pathplanner.lib.PathPlanner;
>>>>>>> Stashed changes
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
<<<<<<< Updated upstream
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
=======
>>>>>>> Stashed changes
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  public final XboxController controller = new XboxController(0);

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
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

<<<<<<< Updated upstream
    Trajectory autoTrajectory = null; // This will be a JSON file created by PathPlanner
=======
    Trajectory p1 = PathPlanner.loadPath("Test", 1.5, 1.5, false);

    Trajectory p2 = PathPlanner.loadPath("Test Inverted", 1.5, 1.5, true);

    return generateRamseteCommand(p1); //.andThen(generateRamseteCommand(p2));
  }
>>>>>>> Stashed changes

  public Command generateRamseteCommand(Trajectory t) {
    RamseteCommand ramsete =
        new RamseteCommand(
            t,
            driveSubsystem::getPose,
            new RamseteController(Constants.AutoConstants.RamseteB, Constants.AutoConstants.RamseteZeta),
            new SimpleMotorFeedforward(
                Constants.AutoConstants.ksVolts,
                Constants.AutoConstants.ksVoltSecondsPerMeter,
                Constants.AutoConstants.kaVoltSecondsSquaredPerMeter),
            Constants.AutoConstants.kinematics,
            driveSubsystem::getWheelSpeeds,
            new PIDController(Constants.AutoConstants.kPDriveVel, 0, 0),
            new PIDController(Constants.AutoConstants.kPDriveVel, 0, 0),
            driveSubsystem::tankDriveVolts,
            driveSubsystem);

    driveSubsystem.resetOdometry(t.getInitialPose());

    return ramsete.andThen(() -> driveSubsystem.tankDriveVolts(0, 0));
  }
}
