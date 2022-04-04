package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShootSubsystem;

public class SetShooterSpin extends CommandBase {

  ShootSubsystem shootSubsystem;
  double primaryVelocity;
  double topVelocity;
  double percentSpin;
  double velocityOffset;
  boolean waitTillAtSpeed;
  int RPM;

  public SetShooterSpin(ShootSubsystem shooter, double vel, double spin, boolean waitToHitSpeed) {
    shootSubsystem = shooter;
    primaryVelocity = (vel / Constants.ShooterConstants.pRPMtoMPSConstant);
    topVelocity = (vel / Constants.ShooterConstants.tRPMtoMPSConstant);
    waitTillAtSpeed = waitToHitSpeed;
    percentSpin = spin/2;

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
