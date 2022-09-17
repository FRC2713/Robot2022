package frc.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxAnalogSensor.Mode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModule {

  CANSparkMax driver;
  CANSparkMax azimuth;
  double offset;

  private final PIDController drivePID = new PIDController(1, 0, 0);

  private final PIDController aziPID = new PIDController(1, 0, 0);

  private final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward aziFF = new SimpleMotorFeedforward(0, 0);

  public SwerveModule(int drivePort, int azimPort, double offset) {
    driver = new CANSparkMax(drivePort, MotorType.kBrushless);
    azimuth = new CANSparkMax(azimPort, MotorType.kBrushless);
    this.offset = offset;

    getDriveEncoder()
        .setPositionConversionFactor(2 * Math.PI * (Constants.DriveConstants.wheelDiameter / 2));
    getAziEncoder().setPositionConversionFactor(2 * Math.PI);

    aziPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  private RelativeEncoder getDriveEncoder() {
    return driver.getEncoder();
  }

  private SparkMaxAnalogSensor getAziEncoder() {
    return azimuth.getAnalog(Mode.kAbsolute);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveEncoder().getVelocity(), new Rotation2d(getAziEncoder().getPosition()));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(getAziEncoder().getPosition()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        drivePID.calculate(getDriveEncoder().getVelocity(), state.speedMetersPerSecond);

    final double driveFeedforward = driveFF.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        aziPID.calculate((getAziEncoder().getPosition() + offset), state.angle.getRadians());

    final double turnFeedforward = aziFF.calculate(aziPID.getSetpoint());

    driver.setVoltage(driveOutput + driveFeedforward);
    azimuth.setVoltage(turnOutput + turnFeedforward);
  }
}
