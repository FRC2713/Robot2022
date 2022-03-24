package frc.robot.commands;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class MoveCircle extends CommandBase {
  private final DriveSubsystem drive;
  private RelativeEncoder encoder;
  private double startingDistance;
  private double startingAngle;
  private int count = 0;

  public MoveCircle(DriveSubsystem drive) {
    this.drive = drive;
    encoder = drive.getRightEncoder();
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    startingAngle = drive.getDegrees();
  }

  @Override
  public void execute() {
    drive.GTADrive(.2, 0, 0.3);
    // drive.GTADrive(0, 0, 0.5);
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
    // count = 0;
  }
}
