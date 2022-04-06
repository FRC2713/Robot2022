package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.List;

public class MotionProfileCommand extends CommandBase {
  private static final double ramseteB = 2;
  private static final double ramseteZeta = 0.7;

  private final DriveSubsystem drive;
  private final DifferentialDriveKinematics kinematics;
  private final Trajectory trajectory;
  private final RamseteController controller = new RamseteController(ramseteB, ramseteZeta);
  private final Timer timer = new Timer();

  private PIDController left = new PIDController(AutoConstants.LeftSide.kP, 0, 0);
  private PIDController right = new PIDController(AutoConstants.RightSide.kP, 0, 0);

  private SimpleMotorFeedforward leftFF =
      new SimpleMotorFeedforward(
          AutoConstants.LeftSide.kS, AutoConstants.LeftSide.kV, AutoConstants.LeftSide.kA);
  private SimpleMotorFeedforward rightFF =
      new SimpleMotorFeedforward(
          AutoConstants.RightSide.kS, AutoConstants.RightSide.kV, AutoConstants.RightSide.kA);

  /**
   * Creates a new MotionProfileCommand with extra constraints. Drives along the specified path
   * based on odometry data.
   */
  public MotionProfileCommand(
      DriveSubsystem drive,
      double startVelocityMetersPerSec,
      List<Pose2d> waypoints,
      double endVelocityMetersPerSec,
      boolean reversed) {
    addRequirements(drive);
    this.drive = drive;

    // Set up trajectory configuration
    kinematics = new DifferentialDriveKinematics(AutoConstants.trackWidth);
    CentripetalAccelerationConstraint centripetalAccelerationConstraint =
        new CentripetalAccelerationConstraint(AutoConstants.maxCentripetalAcceleration);
    TrajectoryConfig config =
        new TrajectoryConfig(AutoConstants.maxSpeed, AutoConstants.maxAccel)
            .setKinematics(kinematics)
            .addConstraint(centripetalAccelerationConstraint)
            .setStartVelocity(startVelocityMetersPerSec)
            .setEndVelocity(endVelocityMetersPerSec)
            .setReversed(reversed);

    config.addConstraint(
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                AutoConstants.BothSides.kS, AutoConstants.BothSides.kV, AutoConstants.BothSides.kA),
            kinematics,
            AutoConstants.maxVoltageApplied));

    // Generate trajectory
    Trajectory generatedTrajectory;
    try {
      generatedTrajectory = TrajectoryGenerator.generateTrajectory(waypoints, config);
    } catch (TrajectoryGenerationException exception) {
      generatedTrajectory = new Trajectory();
    }
    trajectory = generatedTrajectory;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    State setpoint = trajectory.sample(timer.get());
    ChassisSpeeds chassisSpeeds = controller.calculate(drive.getPose(), setpoint);
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

    double leftOut =
        leftFF.calculate(
                drive.getWheelSpeeds().leftMetersPerSecond, wheelSpeeds.leftMetersPerSecond)
            + left.calculate(
                drive.getWheelSpeeds().leftMetersPerSecond, wheelSpeeds.leftMetersPerSecond);
    double rightOut =
        rightFF.calculate(
                drive.getWheelSpeeds().rightMetersPerSecond, wheelSpeeds.rightMetersPerSecond)
            + right.calculate(
                drive.getWheelSpeeds().rightMetersPerSecond, wheelSpeeds.rightMetersPerSecond);

    drive.tankDriveVolts(leftOut, rightOut);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    drive.tankDriveVolts(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }

  public double getDuration() {
    return trajectory.getTotalTimeSeconds();
  }
}
