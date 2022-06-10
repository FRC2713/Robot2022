package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;

public class TurnViaGyro extends TurnInPlace {

  double setpoint;
  double initialReading;

  public TurnViaGyro(DriveSubsystem driveSubsystem, double degrees) {
    super(driveSubsystem);
    initialReading = driveSubsystem.getHeading();
    setpoint = degrees;
  }

  @Override
  public double getMeasurement() {
    SmartDashboard.putNumber("Turn in place meas", (driveSubsystem.getHeading()));
    return (driveSubsystem.getHeading());
  }

  @Override
  public double getSetpoint() {
    SmartDashboard.putNumber("Turn in place setpoint", setpoint);
    return setpoint;
  }
}
