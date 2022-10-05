package frc.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {

  CANSparkMax driver;
  CANSparkMax azimuth;
  double offset;
  SwerveModuleState desiredState;

  OffsetAbsoluteAnalogEncoder azimuthAnalogEncoder;
  private final PIDController drivePID = new PIDController(1, 0, 0);
  private final PIDController aziPID = new PIDController(1, 0, 0);

  private final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward aziFF = new SimpleMotorFeedforward(0, 0);

  public SwerveModule(int drivePort, int azimPort, int azimuthEncoderPort, double offset) {
    driver = new CANSparkMax(drivePort, MotorType.kBrushless);
    azimuth = new CANSparkMax(azimPort, MotorType.kBrushless);

    driver.getEncoder()
        .setPositionConversionFactor(2 * Math.PI * (Constants.DriveConstants.wheelDiameter / 2));
    driver.getAnalog(Mode.kAbsolute).setPositionConversionFactor(2 * Math.PI);

    aziPID.enableContinuousInput(-Math.PI, Math.PI);

    azimuthAnalogEncoder = new OffsetAbsoluteAnalogEncoder(azimuthEncoderPort, offset);
  }

  // private RelativeEncoder getDriveEncoder() {
  //   return driver.getEncoder();
  // }

  // private SparkMaxAnalogSensor getAziEncoder() {
  //   return azimuth.getAnalog(Mode.kAbsolute);
  // }

  public double getAdjustedAzimuthPosition() {
    return azimuth.getAnalog(Mode.kAbsolute).getPosition() - offset;
  }

  private double getDriveVelocity() {
    return driver.getEncoder().getPosition();
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveVelocity(), new Rotation2d(getAdjustedAzimuthPosition() - offset));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState = this.desiredState;
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(getAdjustedAzimuthPosition()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        drivePID.calculate(getDriveVelocity(), state.speedMetersPerSecond);

    final double driveFeedforward = driveFF.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        aziPID.calculate(getAdjustedAzimuthPosition(), state.angle.getRadians());

    final double turnFeedforward = aziFF.calculate(aziPID.getSetpoint());

    driver.setVoltage(driveOutput + driveFeedforward);
    azimuth.setVoltage(turnOutput + turnFeedforward);
  }

  public void update() {
    final double driveOutput =
        drivePID.calculate(getDriveVelocity(), desiredState.speedMetersPerSecond);

    final double driveFeedforward = driveFF.calculate(desiredState.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        aziPID.calculate(getAdjustedAzimuthPosition(), desiredState.angle.getRadians());

    final double turnFeedforward = aziFF.calculate(aziPID.getSetpoint());

    driver.setVoltage(driveOutput + driveFeedforward);
    azimuth.setVoltage(turnOutput + turnFeedforward);
    }
    
    @Override
    public void periodic() {
      update();
    }
}
