// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Constants.DriveConstants;

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
  // ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  AHRS gyro = new AHRS(SerialPort.Port.kUSB);
  private final DifferentialDriveOdometry roboOdometry =
      new DifferentialDriveOdometry(gyro.getRotation2d());

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    left1.restoreFactoryDefaults();
    right1.restoreFactoryDefaults();
    left2.restoreFactoryDefaults();
    right2.restoreFactoryDefaults();
    left1.setSmartCurrentLimit(Constants.DriveConstants.kCurrentLimit);
    right1.setSmartCurrentLimit(Constants.DriveConstants.kCurrentLimit);
    left1.setInverted(true);
    right1.setInverted(false);
    left2.follow(left1);
    right2.follow(right1);
    setHalfBrake();
  
    left1.setIdleMode(IdleMode.kBrake);
    left2.setIdleMode(IdleMode.kCoast);
    right1.setIdleMode(IdleMode.kBrake);
    right2.setIdleMode(IdleMode.kCoast);

    left1.getEncoder().setPositionConversionFactor(DriveConstants.kDistancePerPulse);
    right1.getEncoder().setPositionConversionFactor(DriveConstants.kDistancePerPulse);

    left1.getEncoder().setVelocityConversionFactor(DriveConstants.kDistancePerPulse / 60.0);
    right1.getEncoder().setVelocityConversionFactor(DriveConstants.kDistancePerPulse / 60.0);
  }

  // public DifferentialDrive getRoboDrive() {
  // return roboDrive;
  // }

  public RelativeEncoder getLeftEncoder() {
    return left1.getEncoder();
  }

  public void setAllCoast() {
    left1.setIdleMode(IdleMode.kCoast);
    right1.setIdleMode(IdleMode.kCoast);
    left2.setIdleMode(IdleMode.kCoast);
    right2.setIdleMode(IdleMode.kCoast);
  }

  public void setHalfBrake() {

    left1.setIdleMode(IdleMode.kBrake);
    left2.setIdleMode(IdleMode.kCoast);
    right1.setIdleMode(IdleMode.kBrake);
    right2.setIdleMode(IdleMode.kCoast);
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
    getLeftEncoder().setPosition(0);
    getRightEncoder().setPosition(0);
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

    SmartDashboard.putNumber("Gyro yaw", getHeading());
    SmartDashboard.putNumber("Left Enc", left1.getEncoder().getPosition());
    SmartDashboard.putNumber("Right Enc", right1.getEncoder().getPosition());

    SmartDashboard.putNumber("Odometry X", roboOdometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Odometry Y", roboOdometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Odometry H", roboOdometry.getPoseMeters().getRotation().getDegrees());
  }

  public void GTADrive(double leftTrigger, double rightTrigger, double turn) {
    if (-Constants.DriveConstants.kJoystickTurnDeadzone <= turn
        && turn <= Constants.DriveConstants.kJoystickTurnDeadzone) {
      turn = 0.0;
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
