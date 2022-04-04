package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShootSubsystem;

public class SetShooterSpin extends CommandBase {

  ShootSubsystem shootSubsystem;
  int primaryVelocity;
  int topVelocity;
  double percentSpin;
  double velocityOffset;
  boolean waitTillAtSpeed;
  int RPM;

  public SetShooterSpin(ShootSubsystem shooter, double vel, double spin, boolean waitToHitSpeed) {
    shootSubsystem = shooter;
    primaryVelocity = (int) (vel * Constants.ShooterConstants.pRPMtoMPSConstant);
    topVelocity = (int) (vel * Constants.ShooterConstants.tRPMtoMPSConstant);
    waitTillAtSpeed = waitToHitSpeed;
    percentSpin = spin;

    addRequirements(shootSubsystem);
  }

  @Override
  public void execute() {
    shootSubsystem.setPrimaryRPM(primaryVelocity + (primaryVelocity * percentSpin));
    shootSubsystem.setTopRPM(topVelocity - (topVelocity * percentSpin));
  }

  @Override
  public boolean isFinished() {
    return waitTillAtSpeed
        ? shootSubsystem.primaryCloseEnough() && shootSubsystem.topCloseEnough()
        : true;
  }
}
