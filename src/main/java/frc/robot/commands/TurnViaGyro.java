package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;

public class TurnViaGyro extends TurnInPlace {

  double setpoint;

  public TurnViaGyro(DriveSubsystem driveSubsystem, double degrees) {
    super(driveSubsystem);
    setpoint = degrees;
  }

  @Override
  public double getMeasurement() {
    return driveSubsystem.getDegrees();
  }

  @Override
  public double getSetpoint() {
    return setpoint;
  }
}
