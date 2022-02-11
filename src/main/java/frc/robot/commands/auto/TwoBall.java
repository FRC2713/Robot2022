package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.RamsetA;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeFourBar;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.FieldConstants;
import frc.robot.util.Util;
import java.util.List;

public class TwoBall extends SequentialCommandGroup {
  private static Rotation2d cargoDangleOfApproach = Rotation2d.fromDegrees(180);
  private static Pose2d cargoDPose =
      new Pose2d(FieldConstants.cargoD.getTranslation(), cargoDangleOfApproach);

  private static Trajectory leg1 =
      RamsetA.makeTrajectory(
          0.0, List.of(FieldConstants.StartingPoints.tarmacD, FieldConstants.cargoE), 0.0, false);

  private static Trajectory leg2 =
      RamsetA.makeTrajectory(
          0.0, List.of(FieldConstants.cargoE, FieldConstants.StartingPoints.fenderB), 0.0, true);

  private static Trajectory leg3 =
      RamsetA.makeTrajectory(
          0,
          List.of(FieldConstants.StartingPoints.fenderB, cargoDPose, FieldConstants.cargoG),
          0,
          false);

  public TwoBall(
      DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, IntakeFourBar fourBar) {
    addCommands(
        sequence(
            new DeployIntake(intakeSubsystem, fourBar),
            RamsetA.RamseteSchmoove(leg1, driveSubsystem),
            RamsetA.RamseteSchmoove(leg2, driveSubsystem),
            /* score , */
            RamsetA.RamseteSchmoove(leg3, driveSubsystem),
            RamsetA.RamseteSchmoove(Util.invertTrajectory(leg3), driveSubsystem)
            /* score */ ));
  }
}
