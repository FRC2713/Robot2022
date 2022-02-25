package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RamsetA;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeFourBar;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.SnekSystem;
import frc.robot.util.FieldConstants;
import frc.robot.util.Util;
import java.util.List;

public class FourBall extends SequentialCommandGroup {
  private static Rotation2d cargoDangleOfApproach = Rotation2d.fromDegrees(180);
  private static Pose2d cargoDPose =
      new Pose2d(FieldConstants.cargoD.getTranslation(), cargoDangleOfApproach);

  private static Trajectory leg1 =
      RamsetA.makeTrajectory(
          0.0,
          List.of(
              FieldConstants.StartingPoints.tarmacD,
              FieldConstants.cargoE.transformBy(
                  Util.Geometry.transformFromTranslation(Units.inchesToMeters(6), 0))),
          0.0,
          Units.feetToMeters(5),
          false);

  private static Trajectory leg2 =
      RamsetA.makeTrajectory(
          0.0,
          List.of(FieldConstants.cargoE, FieldConstants.StartingPoints.fenderB),
          0.0,
          Units.feetToMeters(5),
          true);

  private static Trajectory leg3 =
      RamsetA.makeTrajectory(
          0,
          List.of(FieldConstants.StartingPoints.fenderB, cargoDPose, FieldConstants.cargoG),
          0,
          false);

  public FourBall(
      DriveSubsystem driveSubsystem,
      IntakeSubsystem intakeSubsystem,
      IntakeFourBar fourBar,
      ShootSubsystem shootSubsystem,
      SnekSystem snekSystem) {
    addCommands(
        sequence(
            // new DeployIntake(intakeSubsystem, fourBar),
            RamsetA.RamseteSchmoove(leg1, driveSubsystem),
            RamsetA.RamseteSchmoove(leg2, driveSubsystem),
            // new ShootTillEmpty(shootSubsystem, snekSystem),
            RamsetA.RamseteSchmoove(leg3, driveSubsystem),
            RamsetA.RamseteSchmoove(Util.invertTrajectory(leg3), driveSubsystem)));
    // new ShootTillEmpty(shootSubsystem, snekSystem)));
  }
}
