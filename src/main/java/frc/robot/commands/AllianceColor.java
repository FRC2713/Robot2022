package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.StripSubsystem;
import frc.robot.subsystems.StripSubsystem.Pattern;

public class AllianceColor extends CommandBase {

  private Alliance alliance;
  private StripSubsystem strip;

  public AllianceColor() {
    strip = StripSubsystem.getInstance();
    addRequirements(strip);
  }

  @Override
  public void initialize() {
    if (alliance != null) {
      setColor();
    }
  }

  @Override
  public void execute() {
    Alliance newAlliance = DriverStation.getInstance().getAlliance();
    if (newAlliance != alliance) {
      alliance = newAlliance;
      setColor();
    }
  }

  private void setColor() {
    if (alliance == Alliance.Blue) {
      strip.lightUp(Pattern.LightChaseBlue);
    } else if (alliance == Alliance.Red) {
      strip.lightUp(Pattern.LightChaseRed);
    } else {
      strip.lightUp(Pattern.LightChaseGray);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}