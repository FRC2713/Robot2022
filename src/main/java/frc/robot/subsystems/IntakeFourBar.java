package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TunableNumber;
import frc.robot.util.Util;

public class IntakeFourBar extends SubsystemBase {

  private CANSparkMax fourBar;
  private TunableNumber tuningSetpoint = new TunableNumber("Intake/Tuning Setpoint", 0);

  private Debouncer currentIsHigh = new Debouncer(1); // 1 second

  public IntakeFourBar() {
    fourBar = new CANSparkMax(Constants.RobotMap.intakeMotorFourBar, MotorType.kBrushless);

    fourBar.restoreFactoryDefaults();

    fourBar.setInverted(false);

    fourBar.setSmartCurrentLimit((int) Constants.IntakeConstants.fourBarCurrentLimit.get());

    fourBar.setIdleMode(CANSparkMax.IdleMode.kBrake);
    fourBar
        .getPIDController()
        .setSmartMotionMaxVelocity(Constants.IntakeConstants.smartMotionMaxVelocity.get(), 0);
    fourBar
        .getPIDController()
        .setSmartMotionMaxAccel(Constants.IntakeConstants.smartMotionMaxAcceleration.get(), 0);
    fourBar.getPIDController().setSmartMotionMinOutputVelocity(0, 0);
    fourBar
        .getPIDController()
        .setSmartMotionAllowedClosedLoopError(
            Constants.IntakeConstants.smartMotionAllowableError.get(), 0);
    fourBar.getPIDController().setP(Constants.IntakeConstants.kP.get());
    fourBar.getPIDController().setFF(Constants.IntakeConstants.kF.get());
    fourBar.getEncoder().setPositionConversionFactor(Constants.IntakeConstants.fourBarRatio);
    fourBar.getEncoder().setVelocityConversionFactor(Constants.IntakeConstants.fourBarRatio);
  }

  public void setFourBarPosition(double position) {
    fourBar.getPIDController().setReference(position, CANSparkMax.ControlType.kSmartMotion);
  }

  public void setFourBarMotor(double speed) {
    fourBar.set(speed);
  }

  public void zero() {
    fourBar.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/Four Bar Position", fourBar.getEncoder().getPosition());
    SmartDashboard.putNumber("Intake/Four Bar Current Draw", fourBar.getOutputCurrent());

    if (Constants.tuningMode) {
      fourBar.setSmartCurrentLimit((int) Constants.IntakeConstants.fourBarCurrentLimit.get());
      fourBar.setIdleMode(CANSparkMax.IdleMode.kBrake);
      fourBar
          .getPIDController()
          .setSmartMotionMaxVelocity(Constants.IntakeConstants.smartMotionMaxVelocity.get(), 0);
      fourBar
          .getPIDController()
          .setSmartMotionMaxAccel(Constants.IntakeConstants.smartMotionMaxAcceleration.get(), 0);
      fourBar.getPIDController().setSmartMotionMinOutputVelocity(0, 0);
      fourBar
          .getPIDController()
          .setSmartMotionAllowedClosedLoopError(
              Constants.IntakeConstants.smartMotionAllowableError.get(), 0);
      fourBar.getPIDController().setP(Constants.IntakeConstants.kP.get());
      fourBar.getPIDController().setFF(Constants.IntakeConstants.kF.get());
      fourBar
          .getPIDController()
          .setReference(tuningSetpoint.get(), CANSparkMax.ControlType.kSmartMotion);
    }

    if (Util.isWithinAcceptableError(fourBar.getEncoder().getVelocity(), 0, .001)
        && currentIsHigh.calculate(
            fourBar.getOutputCurrent()
                >= (Constants.IntakeConstants.fourBarCurrentLimit.get() - 1))) {
      fourBar.getEncoder().setPosition(0);
    }
  }
}
