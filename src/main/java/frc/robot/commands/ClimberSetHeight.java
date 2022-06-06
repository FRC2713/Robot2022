package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.util.Util;

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
    m_climber.setTargetHeight(height);
  }

  @Override
  public void end(boolean interrupted) {
    // m_climber.setTelescopeSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return Util.isWithinAcceptableError(
            m_climber.getLeftHeight(), height, ClimberConstants.acceptableError.get())
        && Util.isWithinAcceptableError(
            m_climber.getRightHeight(), height, ClimberConstants.acceptableError.get());
  }
}
