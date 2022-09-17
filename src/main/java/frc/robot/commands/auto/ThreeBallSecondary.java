// package frc.robot.commands.auto;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.Constants;
// import frc.robot.commands.AlignToGoal;
// import frc.robot.commands.FinishShot;
// import frc.robot.commands.IntakeExtendToLimit;
// import frc.robot.commands.IntakeSetRollers;
// import frc.robot.commands.LoadSnek;
// import frc.robot.commands.RamsetA;
// import frc.robot.commands.ShootWithLimelight;
// import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.IntakeFourBar;
// import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.LimelightSubsystem;
// import frc.robot.subsystems.ShootSubsystem;
// import frc.robot.subsystems.SnekSystem;
// import frc.robot.subsystems.StripSubsystem;
// import frc.robot.util.FieldConstants;
// import java.util.List;

// public class ThreeBallSecondary extends SequentialCommandGroup {
//   private static Rotation2d cargoDangleOfApproach = Rotation2d.fromDegrees(270);
//   private static Pose2d cargoDPose =
//       new Pose2d(FieldConstants.cargoD.getTranslation(), cargoDangleOfApproach);

//   private static Trajectory leg1 =
//       RamsetA.makeTrajectory(
//           0, List.of(FieldConstants.StartingPoints.fenderA, FieldConstants.cargoB), 0, false);

//   private static Trajectory leg2 =
//       RamsetA.makeTrajectory(
//           0, List.of(FieldConstants.StartingPoints.fenderA, cargoDPose), 0, false);

//   public ThreeBallSecondary(
//       DriveSubsystem driveSubsystem,
//       IntakeSubsystem intakeSubsystem,
//       IntakeFourBar intakeFourBar,
//       ShootSubsystem shootSubsystem,
//       SnekSystem snekSystem,
//       LimelightSubsystem limelightSubsystem,
//       StripSubsystem stripSubsystem) {

//     Command driveToFirstBallAndPickUp =
//         new ParallelDeadlineGroup(
//             RamsetA.RamseteSchmoove(leg1, driveSubsystem, true),
//             new IntakeExtendToLimit(intakeFourBar, 0.25, 15),
//             new IntakeSetRollers(intakeSubsystem, Constants.IntakeConstants.typicalRollerRPM),
//             new LoadSnek(snekSystem));

//     Command driveToThirdBallAndPickUp =
//         new ParallelDeadlineGroup(
//             RamsetA.RamseteSchmoove(leg2, driveSubsystem),
//             new IntakeExtendToLimit(intakeFourBar, 0.25, 15),
//             new IntakeSetRollers(intakeSubsystem, Constants.IntakeConstants.typicalRollerRPM),
//             new LoadSnek(snekSystem));

//     addCommands(
//         driveToFirstBallAndPickUp,
//         new AlignToGoal(driveSubsystem, limelightSubsystem, stripSubsystem),
//         new ShootWithLimelight(shootSubsystem, limelightSubsystem),
//         new FinishShot(snekSystem, shootSubsystem),
//         driveToThirdBallAndPickUp,
//         new AlignToGoal(driveSubsystem, limelightSubsystem, stripSubsystem),
//         new ShootWithLimelight(shootSubsystem, limelightSubsystem),
//         new FinishShot(snekSystem, shootSubsystem));
//   }
// }
