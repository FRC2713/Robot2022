package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SnekSystem;

public class IntakeSetRollersWithProtection extends CommandBase {

  private final IntakeSubsystem intake;
  private final SnekSystem snek; // for sensors
  private double rpm;

  public IntakeSetRollersWithProtection(IntakeSubsystem intake, SnekSystem snek, double rpm) {
    this.intake = intake;
    this.snek = snek;
    this.rpm = rpm;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setRollerRPM(rpm);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return snek.getUpperLimit() && snek.getLowerLimit();
  }
}
