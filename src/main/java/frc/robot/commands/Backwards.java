package frc.robot.commands;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class Backwards extends CommandBase {
  private final DriveSubsystem drive;
  private RelativeEncoder encoder;
  private double startingDistance;

  public Backwards(DriveSubsystem drive) {
    this.drive = drive;
    encoder = drive.getRightEncoder();
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    startingDistance = encoder.getPosition();
  }

  @Override
  public void execute() {
    drive.GTADrive(.1, 0, 0);
  }

  @Override
  public boolean isFinished() {
    return !(Math.abs(encoder.getPosition() - startingDistance) <= 1);
  }
}
