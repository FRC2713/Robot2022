package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.IntakeFourBar;
import frc.robot.subsystems.IntakeSubsystem;

public class StashIntake extends ParallelCommandGroup {

  public StashIntake(IntakeSubsystem intakeSubsystem, IntakeFourBar intakeFourBar) {
    addCommands(new IntakeSetFourBar(intakeFourBar, 0), new IntakeSetRollers(intakeSubsystem, 0));
  }
}
