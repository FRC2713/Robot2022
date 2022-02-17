package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SnekSystem;

public class ForceSnek extends CommandBase {

  SnekSystem snekSystem;

  public ForceSnek(SnekSystem snek) {
    snekSystem = snek;

    addRequirements(snekSystem);
  }

  @Override
  public void execute() {
    snekSystem.setUpperSnekSpeed(Constants.SnekConstants.snekSpeed);
    snekSystem.setLowerSnekSpeed(
        Constants.SnekConstants.snekSpeed); // uncertain if this is a good idea, but i mean it makes sense right?
  }

  @Override
  public void end(boolean interrupted)
  {
    snekSystem.setLowerSnekSpeed(Constants.zero);
    snekSystem.setUpperSnekSpeed(Constants.zero);
  }

  public boolean isFinished() {
    if (snekSystem.getUpperLimit()) {
      return false;
    } else return true;
  }
}
