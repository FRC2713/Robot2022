// package frc.robot.commands.auto;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
// import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.Constants;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.commands.AlignToGoal;
// import frc.robot.commands.FinishShot;
// import frc.robot.commands.IntakeExtendToLimit;
// import frc.robot.commands.IntakeSetRollers;
// import frc.robot.commands.LoadSnek;
// import frc.robot.commands.RamsetA;
// import frc.robot.commands.SetShooterRPM;
// import frc.robot.commands.ShootWithLimelight;
// import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.IntakeFourBar;
// import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.LimelightSubsystem;
// import frc.robot.subsystems.ShootSubsystem;
// import frc.robot.subsystems.SnekSystem;
// import frc.robot.subsystems.StripSubsystem;
// import frc.robot.util.FieldConstants;
// import frc.robot.util.Util;
// import java.util.List;

// public class FiveBall extends SequentialCommandGroup {
//   private static Trajectory leg1 =
//       RamsetA.makeTrajectory(
//           0.0,
//           List.of(
//               FieldConstants.StartingPoints.tarmacD,
//               FieldConstants.cargoE.transformBy(
//                   Util.Geometry.transformFromRotation(Rotation2d.fromDegrees(0)))),
//           0.0,
//           false);

//   public static Pose2d turnPoint =
//       FieldConstants.StartingPoints.tarmacD
//           .transformBy(
//               Util.Geometry.transformFromTranslation(
//                   Units.inchesToMeters(-30), Units.inchesToMeters(10)))
//           .transformBy(Util.Geometry.transformFromRotation(Rotation2d.fromDegrees(-90)));

//   private static Trajectory leg2 =
//       RamsetA.makeTrajectory(0.0, List.of(FieldConstants.cargoE, turnPoint), 0.0, true);

//   private static Trajectory leg34 =
//       RamsetA.makeTrajectory(0, List.of(turnPoint, FieldConstants.cargoD), 0, false);

//   private static Trajectory leg5 =
//       RamsetA.makeTrajectory(
//           0,
//           List.of(
//               FieldConstants.cargoD,
//               FieldConstants.cargoG.transformBy(
//                   Util.Geometry.transformFromTranslation(0, Units.inchesToMeters(0)))),
//           0,
//           false);

//   private static Trajectory leg6 =
//       RamsetA.makeTrajectory(
//           0,
//           List.of(
//               FieldConstants.cargoG,
//               FieldConstants.cargoG.transformBy(
//                   Util.Geometry.transformFromTranslation(
//                       Units.feetToMeters(-10), Units.feetToMeters(0)))),
//           0,
//           true);

//   public static Command scoreAllBalls(
//       SnekSystem snekSystem,
//       ShootSubsystem shootSubsystem,
//       DriveSubsystem driveSubsystem,
//       LimelightSubsystem limelightSubsystem,
//       StripSubsystem stripSubsystem) {
//     return new SequentialCommandGroup(
//         new AlignToGoal(driveSubsystem, limelightSubsystem, stripSubsystem).withTimeout(1),
//         new ShootWithLimelight(shootSubsystem, limelightSubsystem),
//         new FinishShot(snekSystem, shootSubsystem));
//   }

//   public FiveBall(
//       DriveSubsystem driveSubsystem,
//       IntakeSubsystem intakeSubsystem,
//       IntakeFourBar fourBar,
//       ShootSubsystem shootSubsystem,
//       SnekSystem snekSystem,
//       LimelightSubsystem limelightSubsystem,
//       StripSubsystem stripSubsystem) {

//     Command driveToFirstBallAndPickUp =
//         new ParallelDeadlineGroup(
//             RamsetA.RamseteSchmoove(leg1, driveSubsystem, true),
//             new SetShooterRPM(
//                 shootSubsystem,
//                 Constants.ShooterConstants.primaryHighShotSpeed.get(),
//                 Constants.ShooterConstants.topHighShotSpeed.get(),
//                 true),
//             new IntakeExtendToLimit(fourBar, Constants.IntakeConstants.intakeExtensionSpeed),
//             new IntakeSetRollers(intakeSubsystem, Constants.IntakeConstants.typicalRollerRPM),
//             new LoadSnek(snekSystem));

//     Command driveToTarmac =
//         new ParallelRaceGroup(
//             RamsetA.RamseteSchmoove(leg2, driveSubsystem), new LoadSnek(snekSystem));

//     Command driveToThirdBall =
//         new ParallelDeadlineGroup(
//             new SequentialCommandGroup(RamsetA.RamseteSchmoove(leg34, driveSubsystem)),
//             new LoadSnek(snekSystem));

//     Command driveToTerminal =
//         new ParallelRaceGroup(
//             new ParallelCommandGroup(RamsetA.RamseteSchmoove(leg5, driveSubsystem)),
//             new LoadSnek(snekSystem));

//     Command driveBackToTarmac =
//         new ParallelRaceGroup(
//             RamsetA.RamseteSchmoove(leg6, driveSubsystem), new LoadSnek(snekSystem));

//     addCommands(
//         driveToFirstBallAndPickUp,
//         scoreAllBalls(
//             snekSystem, shootSubsystem, driveSubsystem, limelightSubsystem, stripSubsystem),
//         driveToTarmac,
//         driveToThirdBall,
//         scoreAllBalls(
//             snekSystem, shootSubsystem, driveSubsystem, limelightSubsystem, stripSubsystem),
//         driveToTerminal,
//         new WaitForHumanPlayer(
//             AutoConstants.waitForHumanPlayerDuration, snekSystem, driveSubsystem),
//         driveBackToTarmac,
//         scoreAllBalls(
//             snekSystem, shootSubsystem, driveSubsystem, limelightSubsystem, stripSubsystem));
//   }
// }
