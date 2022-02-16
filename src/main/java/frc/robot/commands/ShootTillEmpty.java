package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.SnekSystem;

public class ShootTillEmpty extends SequentialCommandGroup {
    
  public ShootTillEmpty(ShootSubsystem shootSubsystem, SnekSystem snekSystem) {
    addCommands(
        new SetShooterRPM(shootSubsystem, Constants.ShooterConstants.RPM),
        new WaitCommand(0.5),
        new ForceSnek(snekSystem),
        new SetShooterRPM(shootSubsystem, Constants.zero));
  }
}
