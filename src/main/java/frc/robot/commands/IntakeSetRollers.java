package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeSetRollers extends CommandBase {

  private final IntakeSubsystem intake;
  private double rpm;

  public IntakeSetRollers(IntakeSubsystem intake, double rpm) {
    this.intake = intake;
    this.rpm = rpm;

    addRequirements(intake);
  }

  @Override
  public void execute() {
    intake.setRollerRPM(rpm);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
