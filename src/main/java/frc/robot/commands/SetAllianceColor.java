/*
package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.StripSubsystem;
import frc.robot.subsystems.StripSubsystem.Pattern;

public class SetAllianceColor extends CommandBase {

  private Alliance alliance;
  private StripSubsystem strip;

  public SetAllianceColor() {

    addRequirements(strip);

    SmartDashboard.putString("ZZZZZ", "IU AM RUNNING");
  }

  @Override
  public void initialize() {
    if (alliance != null) {
      setColor();
    }
  }

  @Override
  public void execute() {
    Alliance newAlliance = DriverStation.getAlliance();

    if (newAlliance != alliance) {
      alliance = newAlliance;
      setColor();
    }
  }

  private void setColor() {
    if (alliance == Alliance.Blue) {
      strip.setColor(Pattern.LightChaseBlue);
    } else if (alliance == Alliance.Red) {
      strip.setColor(Pattern.LightChaseRed);
    } else {
      strip.setColor(Pattern.LightChaseGray);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
*/