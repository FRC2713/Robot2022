package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.List;

public class ClimberSubsystem extends SubsystemBase {

  private final CANSparkMax left, right;

  public ClimberSubsystem() {
    left = new CANSparkMax(Constants.RobotMap.climberMotorLeft, MotorType.kBrushless);
    right = new CANSparkMax(Constants.RobotMap.climberMotorRight, MotorType.kBrushless);

    configureSpark(
        left,
        Constants.ClimberConstants.leftKF.get(),
        Constants.ClimberConstants.leftKP.get(),
        false);
    configureSpark(
        right,
        Constants.ClimberConstants.rightKF.get(),
        Constants.ClimberConstants.rightKP.get(),
        true);

    left.enableSoftLimit(SoftLimitDirection.kForward, true);
    left.enableSoftLimit(SoftLimitDirection.kReverse, true);
    left.setSoftLimit(SoftLimitDirection.kForward, Constants.ClimberConstants.maximumHeight);
    left.setSoftLimit(SoftLimitDirection.kReverse, Constants.ClimberConstants.minimumHeight);
    right.enableSoftLimit(SoftLimitDirection.kForward, true);
    right.enableSoftLimit(SoftLimitDirection.kReverse, true);
    right.setSoftLimit(SoftLimitDirection.kForward, Constants.ClimberConstants.maximumHeight);
    right.setSoftLimit(SoftLimitDirection.kReverse, Constants.ClimberConstants.minimumHeight);
  }

  private void configureSpark(CANSparkMax spark, double kF, double kP, boolean inverted) {
    spark.restoreFactoryDefaults();
    spark.setIdleMode(IdleMode.kBrake);
    spark.setInverted(inverted);
    spark.getPIDController().setFF(0);
    spark.getPIDController().setI(0);
    spark.getPIDController().setP(kP);
    spark.setSmartCurrentLimit(Constants.ClimberConstants.kCurrentLimit);
  }

  public void setTelescopeSpeed(double speed) {
    List.of(left, right)
        .forEach(
            (spark) -> {
              SmartDashboard.putNumber("climber speed", speed);
              // if at bottom, disallow negative speed
              if (spark.getEncoder().getPosition() <= Constants.ClimberConstants.minimumHeight
                  && speed < 0) {
                spark.set(0);
                return;
              }
              // if at top, disallow positive speed
              if (spark.getEncoder().getPosition() >= Constants.ClimberConstants.maximumHeight
                  && speed > 0) {
                spark.set(0);
                return;
              }

              spark.set(speed);
            });
  }

  public void periodic() {
    SmartDashboard.putNumber("Climber encoder left", left.getEncoder().getPosition());
    SmartDashboard.putNumber("Climber encoder right", right.getEncoder().getPosition());

    left.getPIDController().setP(Constants.ClimberConstants.leftKP.get());
    right.getPIDController().setP(Constants.ClimberConstants.rightKP.get());
  }

  public void resetTelescopeEncoder() {
    left.getEncoder().setPosition(0);
    right.getEncoder().setPosition(0);
  }

  public double getLeftHeight() {
    return left.getEncoder().getPosition();
  }

  public double getRightHeight() {
    return right.getEncoder().getPosition();
  }

  public void setHeight(double height) {
    if (height < Constants.ClimberConstants.minimumHeight
        || height > Constants.ClimberConstants.maximumHeight) {
      System.err.println("Inputted number outside of allowed climb range - aborted");
      return;
    }
    left.getPIDController().setReference(height, ControlType.kPosition);
    right.getPIDController().setReference(height, ControlType.kPosition);
  }
}
