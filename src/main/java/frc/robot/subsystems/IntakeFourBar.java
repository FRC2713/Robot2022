package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TunableNumber;

public class IntakeFourBar extends SubsystemBase {

  private CANSparkMax fourBar, fourBar2;
  private TunableNumber tuningSetpoint = new TunableNumber("Intake/Tuning Setpoint", 0);

  private boolean operatorControlled = false;

  // private LinearFilter currentFilter = LinearFilter.singlePoleIIR(0.25, 0.02);
  private LinearFilter currentFilter = LinearFilter.movingAverage(10);
  private double currentAmperage = 0;

  public IntakeFourBar() {
    fourBar = new CANSparkMax(Constants.RobotMap.intakeMotorFourBar, MotorType.kBrushless);
    fourBar2 = new CANSparkMax(Constants.RobotMap.intakeMotorFourBar2, MotorType.kBrushless);

    fourBar2.restoreFactoryDefaults();
    fourBar.restoreFactoryDefaults();

    fourBar.setInverted(true);
    fourBar2.follow(fourBar, true);

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

    // setOperatorControlled(false);
  }

  public void disablePID() {
    fourBar.getPIDController().setP(0);
    fourBar.getPIDController().setI(0);
    fourBar.getPIDController().setD(0);
    fourBar.getPIDController().setFF(0);
  }

  public void enablePID() {
    fourBar.getPIDController().setP(Constants.IntakeConstants.kP.get());
    fourBar.getPIDController().setI(0);
    fourBar.getPIDController().setD(0);
    fourBar.getPIDController().setFF(Constants.IntakeConstants.kF.get());
  }

  public void setFourBarPosition(double position) {
    fourBar.getPIDController().setReference(position, CANSparkMax.ControlType.kSmartMotion);
  }

  public void resetFilter() {
    currentFilter.reset();
  }

  public void operateFourBar(double input) {
    fourBar.set(input / 10.0);
  }

  public double getFilteredFourBarMotorCurrent() {
    return currentAmperage;
  }

  private double getUnfilteredFourBarMotorCurrent() {
    return fourBar.getOutputCurrent();
  }

  public void setFourBarMotor(double speed) {
    fourBar.getPIDController();
    fourBar.set(speed);
  }

  public void setEncoderPosition(double distance) {
    fourBar.getEncoder().setPosition(distance);
  }

  public double getEncoderPosition() {
    return fourBar.getEncoder().getPosition();
  }

  public void zero() {
    fourBar.getEncoder().setPosition(Constants.zero);
  }

  public boolean getOperatorControlled() {
    return operatorControlled;
  }

  public void setOperatorControlled(boolean enabled) {
    operatorControlled = enabled;
    if (operatorControlled) {
      fourBar.enableSoftLimit(SoftLimitDirection.kForward, false);
      fourBar.enableSoftLimit(SoftLimitDirection.kReverse, false);
    } else {
      zero();
      // fourBar.enableSoftLimit(SoftLimitDirection.kForward, true);
      // fourBar.enableSoftLimit(SoftLimitDirection.kReverse, true);
      // fourBar.setSoftLimit(SoftLimitDirection.kForward,
      // Constants.IntakeConstants.extensionPoint);
      // fourBar.setSoftLimit(SoftLimitDirection.kReverse, Constants.zero);
    }
  }

  public void retractFourBar() {
    double position = fourBar.getEncoder().getPosition();
    this.setFourBarPosition(position - Constants.IntakeConstants.extensionPoint);
  }

  public double getHomePosition() {
    return (fourBar.getEncoder().getPosition() - Constants.IntakeConstants.extensionPoint);
  }

  public double getVelocity() {
    return fourBar.getEncoder().getVelocity();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/Four Bar Position", fourBar.getEncoder().getPosition());
    SmartDashboard.putNumber("Intake/Four Bar Current Draw", fourBar.getOutputCurrent());
    currentAmperage = currentFilter.calculate(getUnfilteredFourBarMotorCurrent());
    SmartDashboard.putNumber("Intake/Filtered Current", currentAmperage);

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
