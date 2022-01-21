package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeSetRollers extends CommandBase {

  private final IntakeSubsystem intake;
  private double speed;

  public IntakeSetRollers(IntakeSubsystem intake, double speed) {
    this.intake = intake;
    this.speed = speed;

    addRequirements(intake);
  }

  @Override
  public void execute() {

    intake.setRollerSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
