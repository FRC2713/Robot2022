package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TunableNumber;

public class IntakeFourBar extends SubsystemBase {

  private CANSparkMax fourBar;
  private double currPosition;
  private TunableNumber tuningSetpoint = new TunableNumber("Intake/Tuning Setpoint", 0);

  public IntakeFourBar() {
    fourBar = new CANSparkMax(Constants.RobotMap.intakeMotorFourBar, MotorType.kBrushless);

    fourBar.restoreFactoryDefaults();

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
  }
}
