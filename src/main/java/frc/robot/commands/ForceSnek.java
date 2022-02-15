package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SnekSystem;

public class ForceSnek extends CommandBase {

  SnekSystem snekSystem;

  public ForceSnek(SnekSystem snek) {
    snekSystem = snek;

    addRequirements(snekSystem);
  }

  @Override
  public void execute() {
    snekSystem.setUpperSnekSpeed(0.5);
    snekSystem.setLowerSnekSpeed(
        0.5); // uncertain if this is a good idea, but i mean it makes sense right?
  }

  @Override
  public boolean isFinished() {
    if (SnekSystem.upperLimit.get()) {
      return false;
    } else return true;
  }
}
