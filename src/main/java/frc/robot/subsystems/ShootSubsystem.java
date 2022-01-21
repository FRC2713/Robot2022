package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShootSubsystem extends SubsystemBase {
  private CANSparkMax fly1 =
      new CANSparkMax(
          Constants.RobotMap.flywheelLeftPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax fly2 =
      new CANSparkMax(
          Constants.RobotMap.flywheelRightPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  private BangBangController bangbang = new BangBangController(25);
  public FlywheelControl flywheelMode = FlywheelControl.BANG_BANG;

  public enum FlywheelControl {
    BANG_BANG,
    PID;
  }

  public ShootSubsystem() {
    fly2.follow(fly1);

    fly1.restoreFactoryDefaults();
    fly2.restoreFactoryDefaults();

    fly1.getEncoder().setVelocityConversionFactor(Constants.ShooterConstants.gearRatio);

    fly1.setIdleMode(IdleMode.kCoast);
    fly2.setIdleMode(IdleMode.kCoast);

    fly1.setSmartCurrentLimit(Constants.ShooterConstants.currentLimit);

    fly1.getPIDController().setP(Constants.ShooterConstants.kP);
    fly1.getPIDController().setFF(Constants.ShooterConstants.kFF);
  }

  public void setTargetRPM(int targetRPM) {
    // stuff :)
    if (flywheelMode == FlywheelControl.PID) {
      fly1.getPIDController().setReference(targetRPM, ControlType.kVelocity);
    } else if (flywheelMode == FlywheelControl.BANG_BANG) {
      bangbang.setSetpoint(targetRPM);
    }
  }

  public void stopFlywheel() { // SCRAM
    fly1.set(0);
  }

  @Override
  public void periodic() {
    if (flywheelMode == FlywheelControl.BANG_BANG) {
      fly1.set(bangbang.calculate(fly1.getEncoder().getVelocity()));
    } else if (flywheelMode == FlywheelControl.PID) {
      // enjoy the funny shooter because it doesn't need code :)
    }
  }
}
