package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.InterpolatingTreeMap;
import frc.robot.util.TunableNumber;
import frc.robot.util.Util;

public class ShootSubsystem extends SubsystemBase {
  private CANSparkMax fly1 =
      new CANSparkMax(
          Constants.RobotMap.flywheelLeftPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax fly2 =
      new CANSparkMax(
          Constants.RobotMap.flywheelRightPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax top1 =
      new CANSparkMax(Constants.RobotMap.flywheelTopLeft, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax top2 =
      new CANSparkMax(
          Constants.RobotMap.flywheelTopRight, CANSparkMaxLowLevel.MotorType.kBrushless);
  private double primarySetpoint = 0;
  private double topSetpoint = 0;
  private SimpleMotorFeedforward primaryFF =
      new SimpleMotorFeedforward(Constants.ShooterConstants.pks, Constants.ShooterConstants.pkv);
  private SimpleMotorFeedforward topFF =
      new SimpleMotorFeedforward(Constants.ShooterConstants.tks, Constants.ShooterConstants.tkv);

  private TunableNumber surfaceSpeed = new TunableNumber("Shooter/SurfaceSpeed", 0);

  private InterpolatingTreeMap<Double, Double> shooterSpeedMap =
      new InterpolatingTreeMap<>() {
        {
          // distance, speed
          put(8.6, 7.0);
          put(5.8, 7.0);
          put(-2.0, 7.75);
          put(-6.5, 8.25);
          put(-10.2, 9.0);
          put(-17.4, 11.0);
          put(-20.6, 13.0);
          put(-24.3, 14.0);
        }
      };

  public enum FlywheelControl {
    BANG_BANG,
    PID;
  }

  public ShootSubsystem() {
    fly1.restoreFactoryDefaults();
    fly2.restoreFactoryDefaults();
    top1.restoreFactoryDefaults();
    top2.restoreFactoryDefaults();

    fly1.setInverted(true);
    fly2.follow(fly1, true);
    top1.setInverted(true);
    top2.follow(top1, true);

    fly1.getEncoder().setVelocityConversionFactor(Constants.ShooterConstants.PrimaryGearRatio);
    top1.getEncoder().setVelocityConversionFactor(Constants.ShooterConstants.TopGearRatio);

    fly1.setIdleMode(IdleMode.kCoast);
    fly2.setIdleMode(IdleMode.kCoast);
    top1.setIdleMode(IdleMode.kCoast);
    top2.setIdleMode(IdleMode.kCoast);

    fly1.setSmartCurrentLimit(Constants.ShooterConstants.currentLimit);
    top1.setSmartCurrentLimit(Constants.ShooterConstants.topCurrentLimit);

    fly1.getPIDController().setP(Constants.ShooterConstants.PrimarykP.get());
    fly1.getPIDController().setFF(Constants.ShooterConstants.PrimarykFF.get());
    fly1.getPIDController().setD(Constants.ShooterConstants.PrimarykD.get());
    top1.getPIDController().setP(Constants.ShooterConstants.TopkP.get());
    top1.getPIDController().setFF(Constants.ShooterConstants.TopkFF.get());
    top1.getPIDController().setD(Constants.ShooterConstants.TopkD.get());

    fly1.setOpenLoopRampRate(Constants.ShooterConstants.rampRate.get());
    top1.setOpenLoopRampRate(Constants.ShooterConstants.rampRate.get());

    fly1.getPIDController().setOutputRange(0, 1);
    top1.getPIDController().setOutputRange(0, 1);
  }

  public void shootAtDistance(double distance) {
    double surfaceSpeed = shooterSpeedMap.get(distance);
    SmartDashboard.putNumber("Interpolated Surface Speed", surfaceSpeed);
    setPrimarySurfaceSpeed(surfaceSpeed);
    setTopSurfaceSpeed(surfaceSpeed);
  }

  public void setPrimarySurfaceSpeed(double surfaceSpeed) {
    setPrimaryRPM(surfaceSpeed / Constants.ShooterConstants.pRPMtoMPSConstant);
  }

  public void setTopSurfaceSpeed(double surfaceSpeed) {
    setTopRPM(surfaceSpeed / Constants.ShooterConstants.tRPMtoMPSConstant);
  }

  public void setPrimaryRPM(double targetRPM) {
    // stuff :)
    primarySetpoint = targetRPM;
    SmartDashboard.putNumber("ShooterSetpoint", targetRPM);
    // fly1.getPIDController().setReference(targetRPM, ControlType.kVelocity);
    fly1.getPIDController()
        .setReference(
            targetRPM,
            ControlType.kVelocity,
            0,
            primaryFF.calculate(targetRPM),
            ArbFFUnits.kVoltage);
  }

  public void setTopRPM(double targetRPM) {
    targetRPM = Math.min(targetRPM, 11000 * ShooterConstants.TopGearRatio * 0.9);

    topSetpoint = targetRPM;
    SmartDashboard.putNumber("TopSetpoint", targetRPM);
    // top1.getPIDController().setReference(targetRPM, ControlType.kVelocity);
    top1.getPIDController()
        .setReference(
            targetRPM, ControlType.kVelocity, 0, topFF.calculate(targetRPM), ArbFFUnits.kVoltage);
  }

  public boolean primaryCloseEnough() {
    return primarySetpoint > 0
        && Util.isWithinAcceptableError(fly1.getEncoder().getVelocity(), primarySetpoint, 100);
  }

  public boolean topCloseEnough() {
    return topSetpoint > 0
        && Util.isWithinAcceptableError(top1.getEncoder().getVelocity(), topSetpoint, 100);
  }

  public void stopFlywheel() { // SCRAM
    fly1.set(Constants.zero);
  }

  @Override
  public void periodic() {
    if (Constants.tuningMode) {
      fly1.getPIDController().setP(Constants.ShooterConstants.PrimarykP.get());
      fly1.getPIDController().setFF(Constants.ShooterConstants.PrimarykFF.get());

      top1.getPIDController().setP(Constants.ShooterConstants.TopkP.get());
      top1.getPIDController().setFF(Constants.ShooterConstants.TopkFF.get());

      double surfaceSpeedDbl = surfaceSpeed.get();
      double primaryVelocity = surfaceSpeedDbl / Constants.ShooterConstants.pRPMtoMPSConstant;
      double topVelocity = surfaceSpeedDbl / Constants.ShooterConstants.tRPMtoMPSConstant;

      if (topVelocity > 11000 * ShooterConstants.TopGearRatio) {
        topVelocity = 11000 * ShooterConstants.TopGearRatio;
      }

      setPrimaryRPM(primaryVelocity);
      setTopRPM(topVelocity);
    }
    SmartDashboard.putNumber("ShooterRPM", fly1.getEncoder().getVelocity());
    SmartDashboard.putNumber("TopShooterRPM", top1.getEncoder().getVelocity());
    SmartDashboard.putNumber("Shooter error", fly1.getEncoder().getVelocity() - primarySetpoint);
    SmartDashboard.putNumber("Top error", top1.getEncoder().getVelocity() - topSetpoint);
    SmartDashboard.putBoolean("TopAtSpeed", topCloseEnough());
    SmartDashboard.putBoolean("PrimAtSpeed", primaryCloseEnough());
  }
}
