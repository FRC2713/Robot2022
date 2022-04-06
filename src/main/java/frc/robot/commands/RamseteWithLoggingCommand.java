package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

public class RamseteWithLoggingCommand extends RamseteCommand {
  private FieldObject2d m_poseLogger;
  private Trajectory m_trajectory;
  private Timer m_timer = new Timer();

  public RamseteWithLoggingCommand(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      RamseteController controller,
      SimpleMotorFeedforward feedforward,
      DifferentialDriveKinematics kinematics,
      Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds,
      PIDController leftController,
      PIDController rightController,
      BiConsumer<Double, Double> outputVolts,
      FieldObject2d referencePoseLogger,
      Subsystem... requirements) {
    super(
        trajectory,
        pose,
        controller,
        feedforward,
        kinematics,
        wheelSpeeds,
        leftController,
        rightController,
        outputVolts,
        requirements);
    m_poseLogger = referencePoseLogger;
    m_trajectory = trajectory;
  }

  public RamseteWithLoggingCommand(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      RamseteController follower,
      DifferentialDriveKinematics kinematics,
      BiConsumer<Double, Double> outputMetersPerSecond,
      FieldObject2d referencePoseLogger,
      Subsystem... requirements) {
    super(trajectory, pose, follower, kinematics, outputMetersPerSecond, requirements);
    m_poseLogger = referencePoseLogger;
    m_trajectory = trajectory;
  }

  @Override
  public void initialize() {
    super.initialize();
    m_timer.reset();
    m_timer.start();

    m_poseLogger.setTrajectory(m_trajectory);
  }

  @Override
  public void execute() {
    super.execute();
    double curTime = m_timer.get();
    State reference = m_trajectory.sample(curTime);

    m_poseLogger.setPose(reference.poseMeters);
  }
}
