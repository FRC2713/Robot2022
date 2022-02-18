package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
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
        false);
  }

  private void configureSpark(CANSparkMax spark, double kF, double kP, boolean inverted) {
    spark.restoreFactoryDefaults();
    spark.setIdleMode(IdleMode.kBrake);
    spark.setInverted(inverted);
    spark.getPIDController().setFF(kF);
    spark.getPIDController().setP(kP);
    spark.setSmartCurrentLimit(Constants.ClimberConstants.kCurrentLimit);
  }

  public void setTelescopeSpeed(double speed) {
    List.of(left, right)
        .forEach(
            (spark) -> {
              // if at bottom, disallow negative speed
              if (spark.getEncoder().getPosition() <= 0 && speed < 0) {
                return;
              }
              // if at top, disallow positive speed
              if (spark.getEncoder().getPosition() >= Constants.ClimberConstants.maximumHeight
                  && speed > 0) {
                return;
              }

              spark.set(speed);
            });
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
    left.getPIDController().setReference(height, ControlType.kSmartMotion);
    right.getPIDController().setReference(height, ControlType.kSmartMotion);
  }
}
