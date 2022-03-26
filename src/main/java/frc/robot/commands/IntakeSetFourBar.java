package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeFourBar;
import frc.robot.util.Util;

public class IntakeSetFourBar extends CommandBase {

  private final IntakeFourBar intake;
  private double position;

  public IntakeSetFourBar(IntakeFourBar intake, double position) {
    this.intake = intake;
    this.position = position;

    addRequirements(intake);
  }

  @Override
  public void execute() {
    System.out.println("Setting fourbar to " + position);
    intake.setFourBarPosition(position);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return Util.isWithinAcceptableError(this.intake.getEncoderPosition(), this.position, 0.01);
  }
}
