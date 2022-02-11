package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.RamsetA;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeFourBar;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.FieldConstants;
import java.util.List;

public class TwoBall extends SequentialCommandGroup {
  public TwoBall(
      DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, IntakeFourBar fourBar) {
    addCommands(
        sequence(
            new DeployIntake(intakeSubsystem, fourBar),
            RamsetA.RamseteSchmoove(
                RamsetA.makeTrajectory(
                    0,
                    List.of(FieldConstants.StartingPoints.tarmacD, FieldConstants.cargoE),
                    0.0,
                    false),
                driveSubsystem),
            RamsetA.RamseteSchmoove(
                RamsetA.makeTrajectory(
                    0.0,
                    List.of(FieldConstants.cargoE, FieldConstants.StartingPoints.fenderB),
                    0.0,
                    false),
                driveSubsystem)));
  }
}
