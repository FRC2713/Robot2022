package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {
  private final Translation2d frontLeftLocation = new Translation2d(0.314, 0.314);
  private final Translation2d frontRightLocation = new Translation2d(0.314, -0.314);
  private final Translation2d backLeftLocation = new Translation2d(-0.314, 0.314);
  private final Translation2d backRightLocation = new Translation2d(-0.314, -0.314);

  private final SwerveModule frontLeft =
      new SwerveModule(
          Constants.RobotMap.frontLeftDrive, Constants.RobotMap.frontLeftAzi, Constants.RobotMap.frontLeftAziEncoder);
  private final SwerveModule frontRight =
      new SwerveModule(
          Constants.RobotMap.frontRightDrive, Constants.RobotMap.frontRightAzi, Constants.RobotMap.frontRightAziEncoder);
  private final SwerveModule backLeft =
      new SwerveModule(
          Constants.RobotMap.backLeftDrive, Constants.RobotMap.backLeftAzi, Constants.RobotMap.backLeftAziEncoder);
  private final SwerveModule backRight =
      new SwerveModule(
          Constants.RobotMap.backRightDrive, Constants.RobotMap.backRightAzi, Constants.RobotMap.backRightAziEncoder);

  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  private final SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(kinematics, gyro.getRotation2d());

  public SwerveSubsystem() {
    gyro.reset();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("RawAziPosition/Front Left", frontLeft.getState().angle.getRadians());
    SmartDashboard.putNumber("RawAziPosition/Front Right", frontRight.getState().angle.getRadians());
    SmartDashboard.putNumber("RawAziPosition/Back Left", backLeft.getState().angle.getRadians());
    SmartDashboard.putNumber("RawAziPosition/Back Right", backRight.getState().angle.getRadians());
  }

  public void drive(double xSpeed, double ySpeed, double angle) {
    SwerveModuleState[] swerveModuleStates =
        kinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, angle, gyro.getRotation2d()));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.DriveConstants.maxSwerveVel);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void updateOdometry() {
    odometry.update(
        gyro.getRotation2d(),
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState());
  }
}
