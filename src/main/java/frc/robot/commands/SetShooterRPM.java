package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShootSubsystem;

public class SetShooterRPM extends CommandBase {

  ShootSubsystem shootSubsystem;
  double primaryRPM;
  double topRPM;
  boolean waitTillAtSpeed;

  public SetShooterRPM(ShootSubsystem shooter, double pRPM, double tRPM, boolean waitToHitSpeed) {
    shootSubsystem = shooter;
    primaryRPM = pRPM;
    topRPM = tRPM;
    waitTillAtSpeed = waitToHitSpeed;

    addRequirements(shootSubsystem);
  }

  @Override
  public void execute() {
    shootSubsystem.setPrimaryRPM(primaryRPM);
    shootSubsystem.setTopRPM(topRPM);
  }

  @Override
  public boolean isFinished() {
    return waitTillAtSpeed
        ? shootSubsystem.primaryCloseEnough() && shootSubsystem.topCloseEnough()
        : true;
  }
}
