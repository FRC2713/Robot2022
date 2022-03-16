package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeFourBar;

public class IntakeExtendToLimit extends CommandBase {
  private final IntakeFourBar intake;
  private double currentLimit = 0;
  private double motorSpeed = 0;
  private Debouncer currentIsHigh = new Debouncer(0.25);

  public IntakeExtendToLimit(IntakeFourBar fourBar, double speed, double currentLim) {
    this.intake = fourBar;
    this.currentLimit = currentLim;
    this.motorSpeed = speed;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    // this.intake.setOperatorControlled(true);
    this.intake.setFourBarMotor(this.motorSpeed);
  }

  @Override
  public boolean isFinished() {
    double fourBarCurrent = this.intake.getFilteredFourBarMotorCurrent();
    boolean isHighCurrent = this.currentIsHigh.calculate(fourBarCurrent > 10);
    boolean isMoving = this.intake.getVelocity() > 0;
    // SmartDashboard.putNumber("IntakeFourBarCurrent", fourBarCurrent);
    // return fourBarCurrent > this.currentLimit;
    return isHighCurrent;
  }

  @Override
  public void end(boolean interrupted) {
    this.intake.setFourBarMotor(0);
    // this.intake.setOperatorControlled(false);
    if (this.intake.getEncoderPosition() > 0.05) {
      this.intake.setEncoderPosition(Constants.IntakeConstants.extensionPoint);
    }
  }
}
