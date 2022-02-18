package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SnekSystem;

public class EmptySnek extends CommandBase {

  SnekSystem snekSystem;

  public EmptySnek(SnekSystem snek) {
    snekSystem = snek;

    addRequirements(snekSystem);
  }

  @Override
  public void execute() {
    snekSystem.setUpperSnekSpeed(Constants.SnekConstants.snekSpeed);
    snekSystem.setLowerSnekSpeed(Constants.SnekConstants.snekSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    snekSystem.setLowerSnekSpeed(Constants.zero);
    snekSystem.setUpperSnekSpeed(Constants.zero);
  }

  public boolean isFinished() {
    if (snekSystem.getUpperLimit() || snekSystem.getLowerLimit()) {
      return false;
    } else {
      return true;
    }
  }
}
