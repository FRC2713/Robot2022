package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Util;

public class ShootSubsystem extends SubsystemBase {
  private CANSparkMax fly1 =
      new CANSparkMax(
          Constants.RobotMap.flywheelLeftPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax fly2 =
      new CANSparkMax(
          Constants.RobotMap.flywheelRightPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  // private BangBangController bangbang = new BangBangController(10); // Margin of error/tolerance
  public FlywheelControl flywheelMode = FlywheelControl.PID;
  private double rpmSetpoint = 0;

  public enum FlywheelControl {
    BANG_BANG,
    PID;
  }

  public ShootSubsystem() {
    fly1.restoreFactoryDefaults();
    fly2.restoreFactoryDefaults();

    fly1.setInverted(true);
    fly2.follow(fly1, true);

    fly1.getEncoder().setVelocityConversionFactor(Constants.ShooterConstants.gearRatio);

    fly1.setIdleMode(IdleMode.kCoast);
    fly2.setIdleMode(IdleMode.kCoast);

    fly1.setSmartCurrentLimit(Constants.ShooterConstants.currentLimit);

    fly1.getPIDController().setP(Constants.ShooterConstants.kP.get());
    fly1.getPIDController().setFF(Constants.ShooterConstants.kFF.get());

    fly1.setOpenLoopRampRate(Constants.ShooterConstants.rampRate.get());

    fly1.getPIDController().setOutputRange(0, 1);
  }

  public void setTargetRPM(double targetRPM) {
    // stuff :)
    rpmSetpoint = targetRPM;
    SmartDashboard.putNumber("ShooterSetpoint", targetRPM);
    if (flywheelMode == FlywheelControl.PID) {
      fly1.getPIDController().setReference(targetRPM, ControlType.kVelocity);
      // } else if (flywheelMode == FlywheelControl.BANG_BANG) {
      //   bangbang.setSetpoint(targetRPM);
    }
  }

  public boolean closeEnough() {
    return Util.isWithinAcceptableError(fly1.getEncoder().getVelocity(), rpmSetpoint, 100);
  }

  public void stopFlywheel() { // SCRAM
    fly1.set(Constants.zero);
  }

  @Override
  public void periodic() {

    fly1.getPIDController().setP(Constants.ShooterConstants.kP.get());
    fly1.getPIDController().setFF(Constants.ShooterConstants.kFF.get());

    if (flywheelMode == FlywheelControl.BANG_BANG) {
      // fly1.set(bangbang.calculate(fly1.getEncoder().getVelocity()));
    } else if (flywheelMode == FlywheelControl.PID) {
      // enjoy the funny shooter because it doesn't need code :)
    }
    SmartDashboard.putNumber("ShooterRPM", fly1.getEncoder().getVelocity());
    SmartDashboard.putNumber("Shooter error", fly1.getEncoder().getVelocity() - rpmSetpoint);
  }
}
