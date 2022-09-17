// package frc.robot.commands.auto;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.Constants;
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

// public class ThreeBallPartnerSecondary extends SequentialCommandGroup {

//   private static Pose2d TarmacReferencePoint =
//       FieldConstants.cargoD.transformBy(
//           Util.Geometry.transformFromTranslation(
//               -Units.inchesToMeters(26), Units.inchesToMeters(0)));

//   private static Trajectory leg1 =
//       RamsetA.makeTrajectory(
//           0, List.of(FieldConstants.StartingPoints.fenderA, TarmacReferencePoint), 0, false);

//   private static Trajectory leg2 =
//       RamsetA.makeTrajectory(0, List.of(TarmacReferencePoint, FieldConstants.cargoB), 0, false);

//   public ThreeBallPartnerSecondary(
//       DriveSubsystem driveSubsystem,
//       IntakeSubsystem intakeSubsystem,
//       IntakeFourBar intakeFourBar,
//       ShootSubsystem shootSubsystem,
//       SnekSystem snekSystem,
//       LimelightSubsystem limelightSubsystem,
//       StripSubsystem stripSubsystem) {

//     Command pushOutIntake =
//         new ParallelCommandGroup(
//             new IntakeExtendToLimit(intakeFourBar, 0.25, 15),
//             new IntakeSetRollers(intakeSubsystem, Constants.IntakeConstants.typicalRollerRPM));

//     Command straightForwardATad = RamsetA.RamseteSchmoove(leg1, driveSubsystem, true);

//     Command driveToFirstBallAndPickUp =
//         new ParallelDeadlineGroup(
//             RamsetA.RamseteSchmoove(leg2, driveSubsystem),
//             new SetShooterRPM(
//                 shootSubsystem,
//                 Constants.ShooterConstants.primaryHighShotSpeed.get(),
//                 Constants.ShooterConstants.topHighShotSpeed.get(),
//                 true),
//             new LoadSnek(snekSystem));

//     addCommands(
//         new WaitCommand(1),
//         pushOutIntake,
//         new WaitCommand(1),
//         straightForwardATad,
//         new AlignToGoal(driveSubsystem, limelightSubsystem, stripSubsystem).withTimeout(1),
//         new ShootWithLimelight(shootSubsystem, limelightSubsystem),
//         new FinishShot(snekSystem, shootSubsystem),
//         driveToFirstBallAndPickUp,
//         new AlignToGoal(driveSubsystem, limelightSubsystem, stripSubsystem).withTimeout(1),
//         new ShootWithLimelight(shootSubsystem, limelightSubsystem),
//         new FinishShot(snekSystem, shootSubsystem));
//   }
// }
