package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeFourBar;
import frc.robot.subsystems.IntakeSubsystem;

public class DeployIntake extends ParallelCommandGroup {
    //j

    public DeployIntake(IntakeSubsystem intakeSubsystem, IntakeFourBar intakeFourBar) {
        addCommands(
            new IntakeSetFourBar(intakeFourBar, Constants.zero), //we have no idea what the fully deployed position is supposed to be :(
            new IntakeSetRollers(intakeSubsystem, Constants.IntakeConstants.speed)
        );
    }
}
