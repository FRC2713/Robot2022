package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ShootSubsystem;

public class ShootALowBall extends SequentialCommandGroup {

  public ShootALowBall(ShootSubsystem shootSubsystem) {
    addCommands(
        new InstantCommand(
            () -> {
              shootSubsystem.setTargetRPM(Constants.ShooterConstants.RPM);
            }),
        new WaitCommand(1),
        new InstantCommand(
            () -> {
              shootSubsystem.setTargetRPM(Constants.zero);
            }));
  }
}
