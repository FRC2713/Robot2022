package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.FinishShot;
import frc.robot.commands.IntakeExtendToLimit;
import frc.robot.commands.IntakeSetRollers;
import frc.robot.commands.LoadSnek;
import frc.robot.commands.RamsetA;
import frc.robot.commands.SetShooterRPM;
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
      new Pose2d(FieldConstants.cargoD.getTranslation(), cargoDangleOfApproach)
          .transformBy(Util.Geometry.transformFromTranslation(0, -Units.inchesToMeters(13)));

  private static Trajectory leg1 =
      RamsetA.makeTrajectory(
          0.0,
          List.of(FieldConstants.StartingPoints.tarmacD, FieldConstants.cargoE),
          0.0,
          Units.feetToMeters(8),
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
          List.of(
              FieldConstants.StartingPoints.fenderB,
              cargoDPose,
              FieldConstants.cargoG.transformBy(
                  Util.Geometry.transformFromTranslation(
                      -Units.inchesToMeters(3), -Units.inchesToMeters(3)))),
          0,
          false);

  private static Trajectory leg4 =
      RamsetA.makeTrajectory(
          0,
          List.of(
              FieldConstants.cargoG,
              FieldConstants.StartingPoints.fenderB.transformBy(
                  Util.Geometry.transformFromTranslation(
                      -Units.inchesToMeters(13),
                      -Units.inchesToMeters(10)))), // 8-13 is probably acceptable
          0,
          true);

  public static Command scoreAllBalls(SnekSystem snekSystem, ShootSubsystem shootSubsystem) {
    return new FinishShot(snekSystem, shootSubsystem);
  }

  public FourBall(
      DriveSubsystem driveSubsystem,
      IntakeSubsystem intakeSubsystem,
      IntakeFourBar fourBar,
      ShootSubsystem shootSubsystem,
      SnekSystem snekSystem) {

    Command driveToFirstBallAndPickUp =
        new ParallelDeadlineGroup(
            RamsetA.RamseteSchmoove(leg1, driveSubsystem),
            new IntakeExtendToLimit(fourBar, Constants.IntakeConstants.intakeExtensionSpeed),
            new IntakeSetRollers(intakeSubsystem, Constants.IntakeConstants.typicalRollerRPM),
            new LoadSnek(snekSystem));

    Command driveToHubFromFirstBall =
        new ParallelRaceGroup(
            new ParallelCommandGroup(
                new SetShooterRPM(
                    shootSubsystem,
                    Constants.ShooterConstants.primaryLowShotSpeed.get(),
                    Constants.ShooterConstants.topLowShotSpeed.get(),
                    true),
                RamsetA.RamseteSchmoove(leg2, driveSubsystem)),
            new LoadSnek(snekSystem));

    Command driveThroughThirdBallToFourth =
        new ParallelDeadlineGroup(
            RamsetA.RamseteSchmoove(leg3, driveSubsystem),
            new LoadSnek(snekSystem),
            new SetShooterRPM(
                shootSubsystem,
                Constants.ShooterConstants.primaryLowShotSpeed.get(),
                Constants.ShooterConstants.topLowShotSpeed.get(),
                true));

    Command driveToHubAgain =
        new ParallelRaceGroup(
            new ParallelCommandGroup(
                new SetShooterRPM(
                    shootSubsystem,
                    Constants.ShooterConstants.primaryLowShotSpeed.get(),
                    Constants.ShooterConstants.topLowShotSpeed.get(),
                    true),
                RamsetA.RamseteSchmoove(leg4, driveSubsystem)),
            new LoadSnek(snekSystem));

    addCommands(
        driveToFirstBallAndPickUp,
        driveToHubFromFirstBall,
        scoreAllBalls(snekSystem, shootSubsystem),
        driveThroughThirdBallToFourth,
        driveToHubAgain,
        scoreAllBalls(snekSystem, shootSubsystem));

    // addCommands(
    // new ParallelCommandGroup(
    // new IntakeExtendToLimit(fourBar, 0.25, 15).perpetually(),
    // sequence(
    // new ParallelDeadlineGroup(
    // sequence(
    // RamsetA.RamseteSchmoove(leg1, driveSubsystem),
    // RamsetA.RamseteSchmoove(leg2, driveSubsystem)),
    // new LoadSnek(snekSystem))),
    // new ShootEverything(snekSystem, shootSubsystem),
    // new ParallelRaceGroup(
    // new LoadSnek(snekSystem),
    // sequence(
    // RamsetA.RamseteSchmoove(leg3, driveSubsystem),
    // RamsetA.RamseteSchmoove(leg4, driveSubsystem))),
    // new ShootEverything(snekSystem, shootSubsystem)));
  }
}
