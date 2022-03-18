package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeFourBar;

public class IntakeExtendToLimit extends CommandBase {
  private final IntakeFourBar intake;
  private double currentLimit = 0;
  private double motorSpeed = 0;
  private Debouncer currentIsHigh =
      new Debouncer(Constants.IntakeConstants.intakeHighCurrentMinimumTime);

  public IntakeExtendToLimit(IntakeFourBar fourBar, double speed, double currentLim) {
    this.intake = fourBar;
    this.currentLimit = currentLim;
    this.motorSpeed = speed;

    addRequirements(intake);
  }

  public IntakeExtendToLimit(IntakeFourBar fourBar, double speed) {
    this(fourBar, speed, Constants.IntakeConstants.intakeExtensionCurrentLimit);
  }

  @Override
  public void initialize() {
    // this.intake.setOperatorControlled(true);
    this.intake.setFourBarMotor(this.motorSpeed);
  }

  @Override
  public boolean isFinished() {
    double fourBarCurrent = this.intake.getFilteredFourBarMotorCurrent();
    boolean isHighCurrent = this.currentIsHigh.calculate(fourBarCurrent > currentLimit);
    return isHighCurrent;
  }

  @Override
  public void end(boolean interrupted) {
    this.intake.setFourBarMotor(0);
    // this.intake.setOperatorControlled(false);
    if (this.intake.getEncoderPosition() > 0.05) {
      this.intake.setEncoderPosition(Constants.IntakeConstants.extensionPoint);
    }
    this.intake.resetFilter();
  }
}
