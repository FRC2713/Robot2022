package frc.robot.commands;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class MoveCircle extends CommandBase {
    private final DriveSubsystem drive;
    private RelativeEncoder encoder;
    private double startingDistance;
    private double startingAngle;

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
    }

    @Override
    public boolean isFinished() {
        return !((drive.getDegrees() - startingAngle) <= 360);
    }
}
