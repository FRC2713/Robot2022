package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.DriveSubsystem;

public class DanceRoutine extends SequentialCommandGroup {
  public DanceRoutine(DriveSubsystem drive) {
    addCommands(
            new Spin(drive),
            new MoveCircle(drive),
            new Forwards(drive),
            new FancySpin(drive),
            new InverseFancySpin(drive)
    );
  }
}
