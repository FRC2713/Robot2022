// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  private CANSparkMax left1 =
      new CANSparkMax(
          Constants.RobotMap.frontLeftMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax right1 =
      new CANSparkMax(
          Constants.RobotMap.frontRightMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax left2 =
      new CANSparkMax(
          Constants.RobotMap.backLeftMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax right2 =
      new CANSparkMax(
          Constants.RobotMap.backRightMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  private final DifferentialDriveOdometry roboOdometry =
      new DifferentialDriveOdometry(gyro.getRotation2d());

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    //    right1.setInverted(true);
    //    right2.setInverted(true); no clue if i need to do this
    left1.restoreFactoryDefaults();
    right1.restoreFactoryDefaults();
    left2.restoreFactoryDefaults();
    right2.restoreFactoryDefaults();
    left2.follow(left1);
    right2.follow(right1);

    left1.setInverted(true);
    right1.setInverted(false);

    left1.setSmartCurrentLimit(30);
    right1.setSmartCurrentLimit(30);

    left1.getEncoder().setPositionConversionFactor(Constants.DriveConstants.distPerPulse);
    right1.getEncoder().setPositionConversionFactor(Constants.DriveConstants.distPerPulse);
    left1.getEncoder().setVelocityConversionFactor(Constants.DriveConstants.distPerPulse / 60);
    right1.getEncoder().setVelocityConversionFactor(Constants.DriveConstants.distPerPulse / 60);
  }

  public RelativeEncoder getLeftEncoder() {
    return left1.getEncoder();
  }

  public RelativeEncoder getRightEncoder() {
    return right1.getEncoder();
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360) * -1;
  }

  public double getDegrees() {
    return gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return gyro.getRate();
  }

  public Pose2d getPose() {
    return roboOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    roboOdometry.resetPosition(pose, gyro.getRotation2d());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        getLeftEncoder().getVelocity(), getRightEncoder().getVelocity());
  }

  public void resetEncoders() {
    getLeftEncoder().setPosition(Constants.zero);
    getRightEncoder().setPosition(Constants.zero);
  }

  public void resetGyro() {
    gyro.reset();
  }

  public double getAverageEncoderDist() {
    return ((getRightEncoder().getPosition() + getLeftEncoder().getPosition()) / 2.0);
  }

  public void tankDriveVolts(double left, double right) {
    left1.setVoltage(left);
    right1.setVoltage(right);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    roboOdometry.update(
        Rotation2d.fromDegrees(getHeading()),
        getLeftEncoder().getPosition(),
        getRightEncoder().getPosition());

        SmartDashboard.putNumber("Heading", getHeading());
        SmartDashboard.putNumber("odo x", roboOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("odo y", roboOdometry.getPoseMeters().getY());
        SmartDashboard.putNumber("odo h", roboOdometry.getPoseMeters().getRotation().getDegrees());
        
        SmartDashboard.putNumber("Left spd", getWheelSpeeds().leftMetersPerSecond);
        SmartDashboard.putNumber("Right spd", getWheelSpeeds().rightMetersPerSecond);
        
  }

  public void GTADrive(double leftTrigger, double rightTrigger, double turn) {
    if (-Constants.DriveConstants.kJoystickTurnDeadzone <= turn
        && turn <= Constants.DriveConstants.kJoystickTurnDeadzone) {
      turn = Constants.zero;
      SmartDashboard.putBoolean("isInDeadband", true);
    } else {
      SmartDashboard.putBoolean("isInDeadband", false);
    }
    turn = turn * turn * Math.signum(turn);

    double left = rightTrigger - leftTrigger + turn;
    double right = rightTrigger - leftTrigger - turn;
    left = Math.min(1.0, Math.max(-1.0, left));
    right = Math.max(-1.0, Math.min(1.0, right));

    this.right1.set(right);
    this.left1.set(left);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
