package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Backwards;
import frc.robot.commands.FancySpin;
import frc.robot.commands.Forwards;
import frc.robot.commands.Spin;
import frc.robot.subsystems.DriveSubsystem;

public class DanceRoutine extends SequentialCommandGroup {
  public DanceRoutine(DriveSubsystem drive) {
    addCommands(
            new Spin(drive),
            new Backwards(drive),
            new Forwards(drive),
            new FancySpin(drive)
    );
  }
}
