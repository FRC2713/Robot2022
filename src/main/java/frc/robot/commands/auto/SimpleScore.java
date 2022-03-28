package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FinishShot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.SnekSystem;

public class SimpleScore extends SequentialCommandGroup {

  public SimpleScore(DriveSubsystem drive, ShootSubsystem shootSubsystem, SnekSystem snekSystem) {
    addCommands(
        new FinishShot(snekSystem, shootSubsystem),
        new RunCommand(
                () -> {
                  drive.GTADrive(-0.1, 0, 0);
                },
                drive)
            .withTimeout(5));
  }
}
