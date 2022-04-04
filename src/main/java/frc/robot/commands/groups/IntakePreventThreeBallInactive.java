package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.IntakeExtendToLimit;
import frc.robot.commands.IntakeSetRollers;
import frc.robot.subsystems.IntakeFourBar;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SnekSystem;

public class IntakePreventThreeBallInactive extends SequentialCommandGroup {
  public IntakePreventThreeBallInactive(
      IntakeSubsystem intakeSubsystem, SnekSystem snekSystem, IntakeFourBar intakeFourBar) {
    addCommands(
        parallel(
            new IntakeSetRollers(intakeSubsystem, Constants.zero),
            new IntakeExtendToLimit(
                intakeFourBar, Constants.IntakeConstants.intakeRetractionSpeed)));
  }
}
