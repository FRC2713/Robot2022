package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;

public class TurnViaGyro extends TurnInPlace {
  /**
   * @param driveSubsystem
   * @param degrees input between -180 and 180
   */
  public TurnViaGyro(
      DriveSubsystem driveSubsystem, double heading) { // degree input between -180 and 180
    super(driveSubsystem);
    this.setpoint = heading;
  }

  @Override
  public double getMeasurement() {
    SmartDashboard.putNumber("Turn in place meas", (driveSubsystem.getHeading()));
    return driveSubsystem.getHeading();
  }

  @Override
  public double getSetpoint() {
    SmartDashboard.putNumber("Turn in place setpoint", this.setpoint);
    return this.setpoint;
  }
}
