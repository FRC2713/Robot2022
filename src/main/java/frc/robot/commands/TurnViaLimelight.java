package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightSubsystem.SnapshotMode;
import frc.robot.subsystems.StripSubsystem;
import frc.robot.subsystems.StripSubsystem.Pattern;

public class TurnViaLimelight extends TurnInPlace {

  LimelightSubsystem limelightSubsystem;

  public TurnViaLimelight(
      DriveSubsystem driveSubsystem,
      LimelightSubsystem limelightSubsystem,
      StripSubsystem stripSubsystem) {
    super(driveSubsystem);
    this.limelightSubsystem = limelightSubsystem;
  }

  @Override
  public void initialize() {
    super.initialize();
    limelightSubsystem.setSnapshotMode(SnapshotMode.TWO_PER_SECOND);
  }

  @Override
  public void execute() {
    if (!limelightSubsystem.hasValidTargets()) {
      stripSubsystem.setColor(Pattern.StrobeRed);
      return;
    } else {
      stripSubsystem.setColor(Pattern.White);
    }
    super.execute();
  }

  @Override
  public void end(boolean interrupted) {
    limelightSubsystem.setSnapshotMode(SnapshotMode.OFF);
    super.end(interrupted);
  }

  @Override
  public double getMeasurement() {
    return limelightSubsystem.getHorizontalOffset();
  }

  @Override
  public double getSetpoint() {
    return 0;
  }
}
