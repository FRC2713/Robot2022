package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class Spin extends CommandBase {
  private final DriveSubsystem drive;
  private double startingAngle;
  private int count = 0;

  public Spin(DriveSubsystem drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    startingAngle = drive.getDegrees();
  }

  @Override
  public void execute() {
    drive.GTADrive(0, 0, 0.5);
    count++;
  }

  @Override
  public void end(boolean interrupted) {
    drive.GTADrive(0, 0, 0);
    count = 0;
  }

  @Override
  public boolean isFinished() {
    return count > 25;
    // return !((drive.getDegrees() - startingAngle) <= 360);
  }
}
