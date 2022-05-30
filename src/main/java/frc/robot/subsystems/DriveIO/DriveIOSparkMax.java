package frc.robot.subsystems.DriveIO;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DriveIOSparkMax implements DriveIO {

  private CANSparkMax left1;
  private CANSparkMax right1;
  private CANSparkMax left2;
  private CANSparkMax right2;

  private ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  private DifferentialDriveOdometry roboOdometry =
      new DifferentialDriveOdometry(gyro.getRotation2d());

  public DriveIOSparkMax() {
    left1.restoreFactoryDefaults();
    right1.restoreFactoryDefaults();
    left2.restoreFactoryDefaults();
    right2.restoreFactoryDefaults();

    left2.follow(left1);
    right2.follow(right1);

    left1.setInverted(true);
    right1.setInverted(false);

    left1.setSmartCurrentLimit(DriveConstants.currentLimit);
    right1.setSmartCurrentLimit(DriveConstants.currentLimit);

    left1.getEncoder().setPositionConversionFactor(Constants.DriveConstants.distPerPulse);
    right1.getEncoder().setPositionConversionFactor(Constants.DriveConstants.distPerPulse);
    left1.getEncoder().setVelocityConversionFactor(Constants.DriveConstants.distPerPulse / 60);
    right1.getEncoder().setVelocityConversionFactor(Constants.DriveConstants.distPerPulse / 60);
  }

  public CANSparkMax getRightMotor() {
    return right1;
  }

  public CANSparkMax getLeftMotor() {
    return left1;
  }

  @Override
  public void updateInputs(DriveInputs inputs) {
    inputs.gyroYawPositionRadians = getHeadingDegrees(inputs);

    inputs.leftVolts = left1.getBusVoltage();
    inputs.rightVolts = right1.getBusVoltage();

    inputs.rightEncPosition = right1.getEncoder().getPosition();
    inputs.leftEncPosition = left1.getEncoder().getPosition();

    inputs.frontLeftCurrent = left1.getOutputCurrent();
    inputs.frontRightCurrent = right1.getOutputCurrent();

    roboOdometry.update(gyro.getRotation2d(), inputs.leftEncPosition, inputs.rightEncPosition);
  }

  @Override
  public double getHeadingDegrees(DriveInputs inputs) {
    return Math.IEEEremainder(gyro.getAngle(), 360) * -1;
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    left1.setVoltage(leftVolts);
    right1.setVoltage(rightVolts);
  }
}
