// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
// liam sais hi :)
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.util.TunableNumber;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final boolean tuningMode = false;
  public static final int zero = 0; // in case you need a zero :)


  public static final double CIRCLE = 360.0;
  public static final class RobotMap {
    // MOTORS

    public static final int frontLeftDrive = 1;
    public static final int backLeftDrive = 2;
    public static final int frontRightDrive = 3;
    public static final int backRightDrive = 4;

    public static final int frontLeftAzi = 100;
    public static final int backLeftAzi = 200;
    public static final int frontRightAzi = 300;
    public static final int backRightAzi = 400;

    public static final int frontLeftAziEncoder = 1000;
    public static final int backLeftAziEncoder = 2000;
    public static final int frontRightAziEncoder = 3000;
    public static final int backRightAziEncoder = 4000;

    public static final double frontLeftOffset = 0;

    public static final double frontRightOffset = 0;

    public static final double backLeftOffset = 0;

    public static final double backRightOffset = 0;
    public static final int flywheelLeftPort = 5;
    public static final int flywheelRightPort = 10;
    public static final int flywheelTopLeft = 13;
    public static final int flywheelTopRight = 55;

    public static final int intakeMotorRollers = 7;
    public static final int intakeMotorRollers2 = 47;
    public static final int intakeMotorFourBar = 6;
    public static final int intakeMotorFourBar2 = 8;

    public static final int lowerSnek = 50;
    public static final int upperSnek = 9;

    public static final int climberMotorRight = 11;
    public static final int climberMotorLeft = 12;

    // DIO

    public static final int snekLowerSwitch = 3;
    public static final int snekUpperSwitch = 1;

    // PWM

    public static final int stripPort = 0;
  }

  public static final class SwerveConstants {
    public static final double MIN_VOLT = 0.0;
    public static final double MAX_VOLT = 5.0;
  }
  public static final class DriveConstants {
    public static final double kJoystickTurnDeadzone = 0.04;
    public static final double wheelDiameter = 5;
    public static final double gearRatio = 60.0 / 11.0 * 28.0 / 20; // 60.0 / 10.0;
    public static final double distPerPulse =
        (1.0 / gearRatio) * Units.inchesToMeters(wheelDiameter) * Math.PI;

    public static final int currentLimit = 65;

    private static final double bumperlessRobotLength = Units.inchesToMeters(26);
    private static final double bumperlessRobotWidth = Units.inchesToMeters(24);
    private static final double bumperThickness = Units.inchesToMeters(3);

    public static final double fullRobotWidth = bumperlessRobotWidth + bumperThickness * 2;
    public static final double fullRobotLength = bumperlessRobotLength + bumperThickness * 2;

    public static final double maxSwerveVel = 3;
    public static final double maxSwerveAzi = Math.PI;
  }

  public static final class IntakeConstants {
    public static final double fourBarRatio = 1.0 / 60.0 * (40.0 / 51.0);
    public static final TunableNumber kP = new TunableNumber("Intake/kP", 0.0);
    public static final TunableNumber kF = new TunableNumber("Intake/kF", 0.002);
    public static final TunableNumber fourBarCurrentLimit =
        new TunableNumber("Intake/4 Bar Current Limit", 15);
    public static final TunableNumber smartMotionMaxVelocity =
        new TunableNumber("Intake/Smart Motion Max Velocity", 1000);
    public static final TunableNumber smartMotionMaxAcceleration =
        new TunableNumber("Intake/Smart Motion Max Acceleration", 1000);
    public static final TunableNumber smartMotionAllowableError =
        new TunableNumber("Intake/Smart Motion Allowable Error", 0.0001);
    public static final float extensionPoint = 0.135f;

    public static final int rollerCurrentLimit = 20;
    public static final double typicalRollerRPM = 2200;
    public static final double spitRollerRPM = 0; // 1100
    public static final double rollerRatio = 12.0 / 60.0;
    public static final double maxRollerRpm = 11000 * rollerRatio;

    public static final double intakeExtensionCurrentLimit = 10;
    public static final double intakeExtensionSpeed = 0.25;
    public static final double intakeRetractionSpeed = -0.25;
    public static final double intakeHighCurrentMinimumTime = 0.25;
  }

  public static final class ShooterConstants {
    public static final double twoBallSpeedOffset = 125;
    public static final double primaryRadius = Units.inchesToMeters(1.5);
    public static final double topRadius = Units.inchesToMeters(.75);

    // MAKE THESEz
    public static final double pks = 0.18554 / 60.0;
    public static final double pkv = 0.12648 / 60.0;
    public static final double tks = 0.33445 / 60.0;
    public static final double tkv = 0.10419 / 60.0;

    public enum GoalType {
      LOW,
      HIGH;
    }

    public static final double PrimaryGearRatio = 1.0;
    public static final double TopGearRatio = 21.0 / 33.0;
    public static final int currentLimit = 85;
    public static final int topCurrentLimit = 40;
    public static final TunableNumber PrimarykP =
        new TunableNumber("Shooter/kP", 0.00000087061 / 60);
    public static final TunableNumber PrimarykFF =
        new TunableNumber("Shooter/kFF", 0.0); // 0.00018);
    public static final TunableNumber PrimarykD = new TunableNumber("Shooter/kD", 0.000);

    // 2.0319E-08
    // 0.000000020319
    public static final TunableNumber TopkP =
        new TunableNumber("TopShooter/kP", 0.00000040308); // 0.0003);
    public static final TunableNumber TopkFF =
        new TunableNumber("TopShooter/kFF", 0.0); // 0.00026);
    public static final TunableNumber TopkD = new TunableNumber("TopShooter/kD", 0.0); // 0.00003);

    public static final TunableNumber primaryLowShotSpeed =
        new TunableNumber("Primary Shooter/Speed", 1300);
    public static final TunableNumber topLowShotSpeed =
        new TunableNumber("Top Shooter/Speed", 1300);
    public static final TunableNumber primaryHighShotSpeed =
        new TunableNumber("Primary Shooter/Speed", 2200);
    public static final TunableNumber topHighShotSpeed =
        new TunableNumber("Top Shooter/Speed", 3600);

    public static final double pRPMtoMPSConstant = (primaryRadius * 2 * 3.14 / 60); // 290;
    public static final double tRPMtoMPSConstant = (topRadius * 2 * 3.14 / 60);

    public static final boolean waitUntilAtSpeed = true;

    public static final TunableNumber rampRate = new TunableNumber("Shooter/Ramp Rate", 0.05);
  }

  public static final class SnekConstants {
    public static final int currentLimit = 20;
    public static final double snekSpeed = 0.4;
    public static final double upperSnekSpeed = 0.2;

    public static final double upperReversePower = -0.4;
    public static final double lowerReversePower = -0.1;
    public static final double reverseDuration = 0.1;

    public static final double debouncerDuration = 0.25;
    public static final double secondHighShotDelay = 0.3;
  }

  public static final class AutoConstants {
    // FF and FB gains; NEED TO BE DETERMINED ON THE FULLY BUILT ROBOT, WILL CHANGE
    // WITH WEIGHT
    public static final double ksVolts = 0.15161; // 0.15166; // 0.20541;
    public static final double kvVoltSecondsPerMeter = 2.511; // 2.5108; // 2.4361;
    public static final double kaVoltSecondsSquaredPerMeter = 0.34892; // 0.34944; // 0.25946;

    public static final double kPDriveVel = 5.7664; // 2.9805; // 3.95;

    // more kinematics stuff
    public static final double trackWidth = Units.inchesToMeters(22);
    public static final DifferentialDriveKinematics kinematics =
        new DifferentialDriveKinematics(trackWidth);

    public static final double maxCentripetalAcceleration = 1.5;

    // Ramsete constants; generally the same on all robots
    public static final double RamseteZeta = 0.7;
    public static final double RamseteB = 2;

    // Max speeds
    public static final double maxSpeed = Units.feetToMeters(15);
    public static final double maxAccel = Units.feetToMeters(10);
    public static final double maxVoltageApplied = 10;

    // five ball
    public static final double waitForHumanPlayerDuration = 0.3;
    public static final double crawlTowardsHumanPlayerVolts = 0.6;
  }

  public static final class ClimberConstants {
    public static final int kCurrentLimit = 40;

    public static final TunableNumber leftKP = new TunableNumber("Climber/Left KP", 0.6);
    public static final TunableNumber leftKF = new TunableNumber("Climber/Left KF", 0.00);
    public static final TunableNumber leftSmartMotionMaxVelocity =
        new TunableNumber("Climber/Left Max Velocity", 1000);
    public static final TunableNumber leftSmartMotionMaxAcceleration =
        new TunableNumber("Climber/Left Max Accel", 1000);

    public static final TunableNumber rightKP = new TunableNumber("Climber/Right KP", 0.6);
    public static final TunableNumber rightKF = new TunableNumber("Climber/Right KF", 0.00);
    public static final TunableNumber rightSmartMotionMaxVelocity =
        new TunableNumber("Climber/Right Max Velocity", 1000);
    public static final TunableNumber rightSmartMotionMaxAcceleration =
        new TunableNumber("Climber/Right Max Accel", 1000);

    public static final double speed = 1.0;

    public static final double lowHeight = 85.0;

    public static final float minimumHeight = 0.0f; // 40.0f / 60.0f * 36.0f; // 24
    public static final float maximumHeight = 135; // 200.0f / 60.0f * 36.0f;
    public static final double midHeight = maximumHeight;
    public static final TunableNumber acceptableError =
        new TunableNumber("Climber/Acceptable Error", 1);

    //   public static final TunableNumber midRungHeight =
    //       new TunableNumber("Climber/Mid Rung Height", 170);
  }

  public static final class LimelightConstants {
    public static final TunableNumber rotationKP =
        new TunableNumber("Limelight/kp", .0190); // .0125, 1.0 change
    public static final TunableNumber rotationKI = new TunableNumber("Limelight/ki", 0.0019);
    public static final TunableNumber rotationIZone = new TunableNumber("Limelight/kIZone", 0);
    public static final TunableNumber rotationalTolerance =
        new TunableNumber("Limelight/Tolerance", 1.5);
    public static final TunableNumber kTurnInPlaceStaticVolts =
        new TunableNumber("Limelight/RotatekS", 0.85);
    // public static final TunableNumber rotationFloor =
    // new TunableNumber("Limelight/RotateFloor", 0.8);
  }
}
