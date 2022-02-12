package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SnekSystem;

public class IntakeSetRollers extends CommandBase {

  private final IntakeSubsystem intake;
  private double speed;
  private DigitalInput lower = SnekSystem.lowerLimit;
  private DigitalInput upper = SnekSystem.upperLimit;

  public IntakeSetRollers(IntakeSubsystem intake, double speed) {
    this.intake = intake;
    this.speed = speed;

    addRequirements(intake);
  }

  @Override
  public void execute() {

    if (lower.get() && upper.get()) {
      intake.setRollerSpeed(Constants.zero);
    } else {
      intake.setRollerSpeed(speed);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
