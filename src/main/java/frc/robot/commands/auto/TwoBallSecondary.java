package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
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

public class TwoBallSecondary extends SequentialCommandGroup {

  private static Trajectory leg1 =
      RamsetA.makeTrajectory(
          0, List.of(FieldConstants.StartingPoints.fenderA, FieldConstants.cargoB), 0, false);

  public TwoBallSecondary(
      DriveSubsystem driveSubsystem,
      IntakeSubsystem intakeSubsystem,
      IntakeFourBar intakeFourBar,
      ShootSubsystem shootSubsystem,
      SnekSystem snekSystem) {

    Command driveToFirstBallAndPickUp =
        new ParallelDeadlineGroup(
            RamsetA.RamseteSchmoove(leg1, driveSubsystem),
            new IntakeExtendToLimit(intakeFourBar, 0.25, 15),
            new IntakeSetRollers(intakeSubsystem, Constants.IntakeConstants.typicalRollerRPM),
            new LoadSnek(snekSystem));

    Command driveToHubFromFirstBall =
        new ParallelRaceGroup(
            new ParallelCommandGroup(
                new SetShooterRPM(
                    shootSubsystem, Constants.ShooterConstants.typicalShotSpeed.get(), true),
                RamsetA.RamseteSchmoove(Util.invertTrajectory(leg1), driveSubsystem)),
            new LoadSnek(snekSystem));
    addCommands(
        driveToFirstBallAndPickUp,
        driveToHubFromFirstBall,
        new FinishShot(snekSystem, shootSubsystem));
  }
}
