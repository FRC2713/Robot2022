package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.SnekSystem;

public class ShootALowBall extends SequentialCommandGroup {

  public ShootALowBall(ShootSubsystem shootSubsystem, SnekSystem snekSystem) {
    addCommands(
        new SetShooterRPM(
            shootSubsystem,
            Constants.ShooterConstants.primaryLowShotSpeed.get(),
            Constants.ShooterConstants.topLowShotSpeed.get(),
            Constants.ShooterConstants.waitUntilAtSpeed),
        new WaitCommand(0.5),
        new ForceSnek(snekSystem),
        new SetShooterRPM(
            shootSubsystem,
            Constants.zero,
            Constants.zero,
            Constants.ShooterConstants.waitUntilAtSpeed));
  }
}
