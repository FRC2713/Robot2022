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

  public static final class RobotMap {
    // MOTORS

    public static final int frontLeftMotorPort = 1;
    public static final int backLeftMotorPort = 2;
    public static final int frontRightMotorPort = 3;
    public static final int backRightMotorPort = 4;

    public static final int flywheelLeftPort = 5;
    public static final int flywheelRightPort = 10;
    public static final int flywheelTopLeft = 13;
    public static final int flywheelTopRight = 55;

    public static final int intakeMotorRollers = 7;
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
  }

  public static final class IntakeConstants {
    public static final double fourBarRatio = 1.0 / 60.0 * (40.0 / 51.0);
    public static final TunableNumber kP = new TunableNumber("Intake/kP", 0.0);
    public static final TunableNumber kF = new TunableNumber("Intake/kF", 0.002);
    public static final TunableNumber fourBarCurrentLimit =
        new TunableNumber("Intake/4 Bar Current Limit", 10);
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

    public enum GoalType {
      LOW,
      HIGH;
    }

    public static final double PrimaryGearRatio = 1.0;
    public static final double TopGearRatio = 12.0 / 33.0;
    public static final int currentLimit = 40;
    public static final TunableNumber PrimarykP = new TunableNumber("Shooter/kP", 0.0005);
    public static final TunableNumber PrimarykFF = new TunableNumber("Shooter/kFF", 0.00018);
    public static final TunableNumber TopkP = new TunableNumber("TopShooter/kP", 0.0003);
    public static final TunableNumber TopkFF = new TunableNumber("TopShooter/kFF", 0.00026);

    public static final TunableNumber primaryLowShotSpeed =
        new TunableNumber("Primary Shooter/Speed", 1700);
    public static final TunableNumber topLowShotSpeed =
        new TunableNumber("Top Shooter/Speed", 1700);
    public static final TunableNumber primaryHighShotSpeed =
        new TunableNumber("Primary Shooter/Speed", 2800);
    public static final TunableNumber topHighShotSpeed =
        new TunableNumber("Top Shooter/Speed", 2900);

    public static final boolean waitUntilAtSpeed = true;

    public static final TunableNumber rampRate = new TunableNumber("Shooter/Ramp Rate", 0.05);
  }

  public static final class SnekConstants {
    public static final int currentLimit = 20;
    public static final double snekSpeed = 0.4;
    public static final double upperSnekSpeed = 0.3;

    public static final double upperReversePower = -0.4;
    public static final double lowerReversePower = -0.1;
    public static final double reverseDuration = 0.25;

    public static final double debouncerDuration = 0.75;
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

    public static final double maxCentripetalAcceleration = 2.54;

    // Ramsete constants; generally the same on all robots
    public static final double RamseteZeta = 0.7;
    public static final double RamseteB = 2;

    // Max speeds
    public static final double maxSpeed = Units.feetToMeters(10);
    public static final double maxAccel = Units.feetToMeters(10);
    public static final double maxVoltageApplied = 10;
  }

  public static final class ClimberConstants {
    public static final int kCurrentLimit = 40;

    public static final TunableNumber leftKP = new TunableNumber("Climber/Left KP", 0.5);
    public static final TunableNumber leftKF = new TunableNumber("Climber/Left KF", 0.00);
    public static final TunableNumber leftSmartMotionMaxVelocity =
        new TunableNumber("Climber/Left Max Velocity", 1000);
    public static final TunableNumber leftSmartMotionMaxAcceleration =
        new TunableNumber("Climber/Left Max Accel", 1000);

    public static final TunableNumber rightKP = new TunableNumber("Climber/Right KP", 0.5);
    public static final TunableNumber rightKF = new TunableNumber("Climber/Right KF", 0.00);
    public static final TunableNumber rightSmartMotionMaxVelocity =
        new TunableNumber("Climber/Right Max Velocity", 1000);
    public static final TunableNumber rightSmartMotionMaxAcceleration =
        new TunableNumber("Climber/Right Max Accel", 1000);

    public static final double speed = 1.0;

    public static final double lowHeight = 80 / 3;
    public static final double midHeight = 200 / 3;

    public static final float minimumHeight = 40.0f / 3;
    public static final float maximumHeight = 200.0f / 3;
    public static final TunableNumber acceptableError =
        new TunableNumber("Climber/Acceptable Error", 1);

    //   public static final TunableNumber midRungHeight =
    //       new TunableNumber("Climber/Mid Rung Height", 170);
  }
}
