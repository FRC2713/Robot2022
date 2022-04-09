package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants.GoalType;
import frc.robot.commands.AlignToGoal;
import frc.robot.commands.FinishShot;
import frc.robot.commands.IntakeExtendToLimit;
import frc.robot.commands.IntakeSetRollers;
import frc.robot.commands.LoadSnek;
import frc.robot.commands.RamsetA;
import frc.robot.commands.SetShooterRPM;
import frc.robot.commands.ShootWithLimelight;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeFourBar;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.SnekSystem;
import frc.robot.subsystems.StripSubsystem;
import frc.robot.util.FieldConstants;
import frc.robot.util.TunableNumber;
import frc.robot.util.Util;
import java.util.List;

public class FiveBall extends SequentialCommandGroup {

  private static TunableNumber topShotSpeed;
  private static TunableNumber primaryShotSpeed;

  private static Trajectory leg1 =
      RamsetA.makeTrajectory(
          0.0, List.of(FieldConstants.StartingPoints.tarmacD, FieldConstants.cargoE), 0.0, false);

  private static Trajectory leg2 =
      RamsetA.makeTrajectory(
          0.0, List.of(FieldConstants.cargoE, FieldConstants.StartingPoints.tarmacD), 0.0, true);

  private static Trajectory leg3 =
      RamsetA.makeTrajectory(
          0,
          List.of(FieldConstants.StartingPoints.tarmacD, FieldConstants.StartingPoints.fenderB),
          0.0,
          true);

  private static Trajectory leg4 =
      RamsetA.makeTrajectory(
          0, List.of(FieldConstants.StartingPoints.fenderB, FieldConstants.cargoD), 0.0, false);

  private static Trajectory leg34 =
      RamsetA.makeTrajectory(
          0, List.of(FieldConstants.StartingPoints.tarmacD, FieldConstants.cargoD), 0, false);

  private static Trajectory leg5 =
      RamsetA.makeTrajectory(0, List.of(FieldConstants.cargoD, FieldConstants.cargoG), 0, false);

  public static Command scoreAllBalls(
      SnekSystem snekSystem,
      ShootSubsystem shootSubsystem,
      DriveSubsystem driveSubsystem,
      LimelightSubsystem limelightSubsystem,
      StripSubsystem stripSubsystem) {
    return new SequentialCommandGroup(
        new AlignToGoal(driveSubsystem, limelightSubsystem, stripSubsystem),
        new ShootWithLimelight(shootSubsystem, limelightSubsystem),
        new FinishShot(snekSystem, shootSubsystem));
  }

  public FiveBall(
      DriveSubsystem driveSubsystem,
      IntakeSubsystem intakeSubsystem,
      IntakeFourBar fourBar,
      ShootSubsystem shootSubsystem,
      SnekSystem snekSystem,
      LimelightSubsystem limelightSubsystem,
      StripSubsystem stripSubsystem,
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
            RamsetA.RamseteSchmoove(leg1, driveSubsystem, true),
            new IntakeExtendToLimit(fourBar, Constants.IntakeConstants.intakeExtensionSpeed),
            new IntakeSetRollers(intakeSubsystem, Constants.IntakeConstants.typicalRollerRPM),
            new LoadSnek(snekSystem));

    Command driveToTarmac =
        new ParallelRaceGroup(
            RamsetA.RamseteSchmoove(leg2, driveSubsystem), new LoadSnek(snekSystem));

    Command driveToFenderThenThirdBall =
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                RamsetA.RamseteSchmoove(leg3, driveSubsystem),
                RamsetA.RamseteSchmoove(leg4, driveSubsystem)),
            new LoadSnek(snekSystem));

    Command driveToThirdBall =
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(RamsetA.RamseteSchmoove(leg34, driveSubsystem)),
            new LoadSnek(snekSystem),
            new SetShooterRPM(shootSubsystem, primaryShotSpeed.get(), topShotSpeed.get(), true));

    Command driveToTerminal =
        new ParallelRaceGroup(
            new ParallelCommandGroup(RamsetA.RamseteSchmoove(leg5, driveSubsystem)),
            new LoadSnek(snekSystem));

    Command driveBackToTarmac =
        new ParallelRaceGroup(
            RamsetA.RamseteSchmoove(Util.invertTrajectory(leg5), driveSubsystem),
            new LoadSnek(snekSystem));

    addCommands(
        driveToFirstBallAndPickUp,
        driveToTarmac,
        scoreAllBalls(
            snekSystem, shootSubsystem, driveSubsystem, limelightSubsystem, stripSubsystem),
        // driveToFenderThenThirdBall,
        driveToThirdBall,
        scoreAllBalls(
            snekSystem, shootSubsystem, driveSubsystem, limelightSubsystem, stripSubsystem),
        driveToTerminal,
        driveBackToTarmac,
        scoreAllBalls(
            snekSystem, shootSubsystem, driveSubsystem, limelightSubsystem, stripSubsystem));
  }
}
