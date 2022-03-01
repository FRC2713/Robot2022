package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.RamsetA;
import frc.robot.commands.ShootTillEmpty;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeFourBar;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.SnekSystem;
import frc.robot.util.FieldConstants;
import frc.robot.util.Util;
import java.util.List;

public class TwoBallSecondarySide extends SequentialCommandGroup {

  private static Trajectory leg1 =
      RamsetA.makeTrajectory(
          0, List.of(FieldConstants.StartingPoints.fenderA, FieldConstants.cargoB), 0, false);

  public TwoBallSecondarySide(
      DriveSubsystem driveSubsystem,
      IntakeSubsystem intakeSubsystem,
      IntakeFourBar intakeFourBar,
      ShootSubsystem shootSubsystem,
      SnekSystem snekSystem) {
    addCommands(
        sequence(
            new DeployIntake(intakeSubsystem, intakeFourBar),
            RamsetA.RamseteSchmoove(leg1, driveSubsystem),
            RamsetA.RamseteSchmoove(Util.invertTrajectory(leg1), driveSubsystem),
            new ShootTillEmpty(shootSubsystem, snekSystem)));
  }
}
