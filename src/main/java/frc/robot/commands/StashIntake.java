package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeFourBar;
import frc.robot.subsystems.IntakeSubsystem;

public class StashIntake extends ParallelCommandGroup {

  public StashIntake(IntakeSubsystem intakeSubsystem, IntakeFourBar intakeFourBar) {
    addCommands(
        new IntakeSetFourBar(
            intakeFourBar,
            -0.1), // we have no idea what the fully stashed position is supposed to be :(
        new IntakeSetRollers(
            intakeSubsystem,
            Constants.IntakeConstants.speed) // no idea what a good speed is, test this please :)
        );
  }
}
