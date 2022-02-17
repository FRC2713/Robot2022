package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShootSubsystem;

public class SetShooterRPM extends CommandBase {

  ShootSubsystem shootSubsystem;
  int RPM;
  boolean waitTillAtSpeed;

  public SetShooterRPM(ShootSubsystem shooter, int speedRPM, boolean waitToHitSpeed) {
    shootSubsystem = shooter;
    RPM = speedRPM;
    waitTillAtSpeed = waitToHitSpeed;

    addRequirements(shootSubsystem);
  }

  @Override
  public void execute() {
    shootSubsystem.setTargetRPM(RPM);
  }

  @Override
  public boolean isFinished() {
    return waitTillAtSpeed ? shootSubsystem.closeEnough() : true;
  }
}
