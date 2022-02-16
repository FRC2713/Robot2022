package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShootSubsystem;

public class SetShooterRPM extends CommandBase {

  ShootSubsystem shootSubsystem;
  int RPM;

  public SetShooterRPM(ShootSubsystem shooter, int speedRPM) {
    shootSubsystem = shooter;
    RPM = speedRPM;

    addRequirements(shootSubsystem);
  }

  @Override
  public void execute() {
    shootSubsystem.setTargetRPM(RPM);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
