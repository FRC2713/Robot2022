package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.List;

public class RamsetA extends SequentialCommandGroup {

  protected Trajectory autoTrajectory;

  public static Trajectory makeTrajectory(
      double startVelocity, List<Pose2d> waypoints, double endVelocity, boolean reversed) {
    return makeTrajectory(
        startVelocity, waypoints, endVelocity, Constants.AutoConstants.maxSpeed, reversed);
  }

  public static Trajectory makeTrajectory(
      double startVelocity,
      List<Pose2d> waypoints,
      double endVelocity,
      double maxVelocity,
      boolean reversed) {
    CentripetalAccelerationConstraint centripetalAccelerationConstraint =
        new CentripetalAccelerationConstraint(Constants.AutoConstants.maxCentripetalAcceleration);
    return TrajectoryGenerator.generateTrajectory(
        waypoints,
        new TrajectoryConfig(
                Math.min(maxVelocity, Constants.AutoConstants.maxSpeed),
                Constants.AutoConstants.maxAccel)
            .setStartVelocity(startVelocity)
            .setEndVelocity(endVelocity)
            .setReversed(reversed)
            .addConstraint(centripetalAccelerationConstraint)
            .addConstraint(
                new DifferentialDriveVoltageConstraint(
                    new SimpleMotorFeedforward(
                        Constants.AutoConstants.ksVolts,
                        Constants.AutoConstants.kvVoltSecondsPerMeter,
                        Constants.AutoConstants.kaVoltSecondsSquaredPerMeter),
                    Constants.AutoConstants.kinematics,
                    Constants.AutoConstants.maxVoltageApplied)));
  }

  public static Trajectory makeTrajectory(
      double startVelocity,
      Pose2d start,
      List<Translation2d> midpoints,
      Pose2d end,
      double endVelocity,
      double maxVelocity,
      boolean reversed) {
    CentripetalAccelerationConstraint centripetalAccelerationConstraint =
        new CentripetalAccelerationConstraint(Constants.AutoConstants.maxCentripetalAcceleration);
    return TrajectoryGenerator.generateTrajectory(
        start,
        midpoints,
        end,
        new TrajectoryConfig(
                Math.min(maxVelocity, Constants.AutoConstants.maxSpeed),
                Constants.AutoConstants.maxAccel)
            .setStartVelocity(startVelocity)
            .setEndVelocity(endVelocity)
            .setReversed(reversed)
            .addConstraint(centripetalAccelerationConstraint)
            .addConstraint(
                new DifferentialDriveVoltageConstraint(
                    new SimpleMotorFeedforward(
                        Constants.AutoConstants.ksVolts,
                        Constants.AutoConstants.kvVoltSecondsPerMeter,
                        Constants.AutoConstants.kaVoltSecondsSquaredPerMeter),
                    Constants.AutoConstants.kinematics,
                    Constants.AutoConstants.maxVoltageApplied)));
  }

  public static Command RamseteSchmoove(Trajectory autoTrajectory, DriveSubsystem driveSubsystem) {
    RamseteCommand ramsete =
        new RamseteCommand(
            autoTrajectory,
            driveSubsystem::getPose,
            new RamseteController(
                Constants.AutoConstants.RamseteB, Constants.AutoConstants.RamseteZeta),
            new SimpleMotorFeedforward(
                Constants.AutoConstants.ksVolts,
                Constants.AutoConstants.kvVoltSecondsPerMeter,
                Constants.AutoConstants.kaVoltSecondsSquaredPerMeter),
            Constants.AutoConstants.kinematics,
            driveSubsystem::getWheelSpeeds,
            new PIDController(Constants.AutoConstants.kPDriveVel, 0, 0),
            new PIDController(Constants.AutoConstants.kPDriveVel, 0, 0),
            driveSubsystem::tankDriveVolts,
            driveSubsystem);

    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              driveSubsystem.resetOdometry(autoTrajectory.getInitialPose());
            }),
        ramsete,
        new InstantCommand(
            () -> {
              driveSubsystem.tankDriveVolts(Constants.zero, Constants.zero);
            }));
  }
}
