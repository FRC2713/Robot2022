package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.IntakeExtendToLimit;
import frc.robot.commands.IntakeSetRollers;
import frc.robot.commands.IntakeSetRollersWithProtection;
import frc.robot.subsystems.IntakeFourBar;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SnekSystem;

public class IntakePreventThreeBallActive extends SequentialCommandGroup {
  public IntakePreventThreeBallActive(
      IntakeSubsystem intakeSubsystem, SnekSystem snekSystem, IntakeFourBar intakeFourBar) {
    addCommands(
        parallel(
            new IntakeExtendToLimit(intakeFourBar, Constants.IntakeConstants.intakeExtensionSpeed),
            new IntakeSetRollersWithProtection(
                intakeSubsystem, snekSystem, Constants.IntakeConstants.typicalRollerRPM)),
        new IntakeSetRollers(intakeSubsystem, Constants.IntakeConstants.spitRollerRPM));
  }
}
