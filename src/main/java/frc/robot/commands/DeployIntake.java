package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.IntakeFourBar;
import frc.robot.subsystems.IntakeSubsystem;

public class DeployIntake extends ParallelCommandGroup {
    //j

    public DeployIntake(IntakeSubsystem intakeSubsystem, IntakeFourBar intakeFourBar) {
        addCommands(
            new IntakeSetFourBar(intakeFourBar, 0), //we have no idea what the fully deployed position is supposed to be :(
            new IntakeSetRollers(intakeSubsystem, 3) //no idea what a good speed is, test this please :)
        );
    }
}
