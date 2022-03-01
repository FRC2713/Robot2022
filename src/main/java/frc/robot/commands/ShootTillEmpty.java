package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.SnekSystem;

public class ShootTillEmpty extends SequentialCommandGroup {

  public ShootTillEmpty(ShootSubsystem shootSubsystem, SnekSystem snekSystem) {
    addCommands(
        new SetShooterRPM(shootSubsystem, Constants.ShooterConstants.typicalShotSpeed.get(), true),
        new WaitCommand(1.5),
        new EmptySnek(snekSystem),
        new SetShooterRPM(shootSubsystem, Constants.zero, false));
  }
}
