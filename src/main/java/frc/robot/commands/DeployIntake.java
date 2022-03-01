package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeFourBar;
import frc.robot.subsystems.IntakeSubsystem;

public class DeployIntake extends ParallelCommandGroup {
  // j

  public DeployIntake(IntakeSubsystem intakeSubsystem, IntakeFourBar intakeFourBar) {
    addCommands(
        new IntakeSetFourBar(intakeFourBar, Constants.IntakeConstants.extensionPoint),
        new IntakeSetRollers(intakeSubsystem, Constants.IntakeConstants.typicalRollerRPM));
  }
}
