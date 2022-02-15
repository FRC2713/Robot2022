package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberSetHeight extends CommandBase {

  public final ClimberSubsystem m_climber;
  public double height;

  public ClimberSetHeight(ClimberSubsystem climber, double height) {
    m_climber = climber;
    this.height = height;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_climber.setHeight(height);
  }

  @Override
  public void end(boolean interrupted) {
    m_climber.setTelescopeSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return (height
        == m_climber
            .getHeight()); /*Subject to change but like it's a thing that like is supposed to be a thing that like stops it when its at the right place.
                           Probably, definitely going to be more complicated and this one is bad and would break something*/
  }
}
