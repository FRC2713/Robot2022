package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants.GoalType;
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
import frc.robot.util.TunableNumber;
import frc.robot.util.Util;
import java.util.List;

public class TwoBallSecondary extends SequentialCommandGroup {

  private static TunableNumber topShotSpeed;
  private static TunableNumber primaryShotSpeed;

  private static Trajectory leg1 =
      RamsetA.makeTrajectory(
          0, List.of(FieldConstants.StartingPoints.fenderA, FieldConstants.cargoB), 0, false);

  private static Trajectory leg2 =
      RamsetA.makeTrajectory(
          0,
          List.of(
              FieldConstants.cargoB,
              FieldConstants.StartingPoints.fenderA.transformBy(
                  Util.Geometry.transformFromTranslation(
                      Units.inchesToMeters(0), -Units.inchesToMeters(13)))),
          0,
          true);

  public TwoBallSecondary(
      DriveSubsystem driveSubsystem,
      IntakeSubsystem intakeSubsystem,
      IntakeFourBar intakeFourBar,
      ShootSubsystem shootSubsystem,
      SnekSystem snekSystem,
      GoalType goalType) {

    if (goalType == GoalType.LOW) {
      topShotSpeed = Constants.ShooterConstants.topLowShotSpeed;
      primaryShotSpeed = Constants.ShooterConstants.primaryLowShotSpeed;
    } else {
      topShotSpeed = Constants.ShooterConstants.topHighShotSpeed;
      primaryShotSpeed = Constants.ShooterConstants.primaryHighShotSpeed;
    }

    Command driveToFirstBallAndPickUp =
        new ParallelDeadlineGroup(
            RamsetA.RamseteSchmoove(leg1, driveSubsystem),
            new IntakeExtendToLimit(intakeFourBar, 0.25, 15),
            new IntakeSetRollers(intakeSubsystem, Constants.IntakeConstants.typicalRollerRPM),
            new LoadSnek(snekSystem));

    Command driveToHubFromFirstBall =
        new ParallelRaceGroup(
            new ParallelCommandGroup(
                new SetShooterRPM(shootSubsystem, primaryShotSpeed.get(), topShotSpeed.get(), true),
                RamsetA.RamseteSchmoove(leg2, driveSubsystem)),
            new LoadSnek(snekSystem));
    addCommands(
        driveToFirstBallAndPickUp,
        driveToHubFromFirstBall,
        new FinishShot(snekSystem, shootSubsystem));
  }
}
