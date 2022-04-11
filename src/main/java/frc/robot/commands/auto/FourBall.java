package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
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
import frc.robot.util.Util;
import java.util.List;

public class FourBall extends SequentialCommandGroup {

  private static Trajectory leg1 =
      RamsetA.makeTrajectory(
          0.0,
          List.of(FieldConstants.StartingPoints.tarmacC, FieldConstants.cargoD),
          0.0,
          Units.feetToMeters(8),
          false);
          
  private static Trajectory leg3 =
      RamsetA.makeTrajectory(
          0,
          List.of(
              FieldConstants.cargoD,
              FieldConstants.cargoG.transformBy(
                  Util.Geometry.transformFromTranslation(
                      -Units.inchesToMeters(0), -Units.inchesToMeters(0)))),
          0,
          false);

  private static Trajectory leg4 =
      RamsetA.makeTrajectory(0, List.of(FieldConstants.cargoG, FieldConstants.cargoD), 0, true);

  public static Command scoreAllBalls(
      SnekSystem snekSystem,
      ShootSubsystem shootSubsystem,
      DriveSubsystem driveSubsystem,
      LimelightSubsystem limelightSubsystem,
      StripSubsystem stripSubsystem) {
    return new SequentialCommandGroup(
        new AlignToGoal(driveSubsystem, limelightSubsystem, stripSubsystem).withTimeout(1),
        new ShootWithLimelight(shootSubsystem, limelightSubsystem),
        new FinishShot(snekSystem, shootSubsystem));
  }

  public FourBall(
      DriveSubsystem driveSubsystem,
      IntakeSubsystem intakeSubsystem,
      IntakeFourBar fourBar,
      ShootSubsystem shootSubsystem,
      SnekSystem snekSystem,
      LimelightSubsystem limelightSubsystem,
      StripSubsystem stripSubsystem) {

    Command driveToFirstBallAndPickUp =
        new ParallelDeadlineGroup(
            RamsetA.RamseteSchmoove(leg1, driveSubsystem, true),
            new IntakeExtendToLimit(fourBar, Constants.IntakeConstants.intakeExtensionSpeed),
            new IntakeSetRollers(intakeSubsystem, Constants.IntakeConstants.typicalRollerRPM),
            new LoadSnek(snekSystem));

    Command driveToTerminal =
        new ParallelDeadlineGroup(
            RamsetA.RamseteSchmoove(leg3, driveSubsystem), new LoadSnek(snekSystem));

    Command driveBackToShotPoint =
        new ParallelRaceGroup(
            new ParallelCommandGroup(
                new SetShooterRPM(
                    shootSubsystem,
                    Constants.ShooterConstants.primaryHighShotSpeed.get(),
                    Constants.ShooterConstants.topHighShotSpeed.get(),
                    true),
                RamsetA.RamseteSchmoove(leg4, driveSubsystem)),
            new LoadSnek(snekSystem));

    addCommands(
        driveToFirstBallAndPickUp,
        scoreAllBalls(
            snekSystem, shootSubsystem, driveSubsystem, limelightSubsystem, stripSubsystem),
        driveToTerminal,
        new WaitForHumanPlayer(
            Constants.AutoConstants.waitForHumanPlayerDuration, snekSystem, driveSubsystem),
        driveBackToShotPoint,
        scoreAllBalls(
            snekSystem, shootSubsystem, driveSubsystem, limelightSubsystem, stripSubsystem));
  }
}
