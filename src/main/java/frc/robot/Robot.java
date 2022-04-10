// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auto.FourBall;
import frc.robot.commands.auto.SimpleScore;
import frc.robot.commands.auto.ThreeBallSecondary;
import frc.robot.commands.auto.TwoBallSecondary;
import frc.robot.subsystems.LimelightSubsystem.LedMode;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private RobotContainer m_robotContainer = new RobotContainer(); // DO NOT DELETE THIS

  private SendableChooser<Command> autoSelect = new SendableChooser<>();

  private Command fourBallLowAuto =
      new FourBall(
              RobotContainer.driveSubsystem,
              RobotContainer.robotIntake,
              RobotContainer.fourBar,
              RobotContainer.shootSubsystem,
              RobotContainer.snekSystem,
              Constants.ShooterConstants.GoalType.LOW)
          .andThen(
              () -> RobotContainer.driveSubsystem.tankDriveVolts(Constants.zero, Constants.zero));

  private Command fourBallHighAuto =
      new FourBall(
              RobotContainer.driveSubsystem,
              RobotContainer.robotIntake,
              RobotContainer.fourBar,
              RobotContainer.shootSubsystem,
              RobotContainer.snekSystem,
              Constants.ShooterConstants.GoalType.HIGH)
          .andThen(
              () -> RobotContainer.driveSubsystem.tankDriveVolts(Constants.zero, Constants.zero));

  private Command threeBallLowAuto =
      new ThreeBallSecondary(
              RobotContainer.driveSubsystem,
              RobotContainer.robotIntake,
              RobotContainer.fourBar,
              RobotContainer.shootSubsystem,
              RobotContainer.snekSystem,
              Constants.ShooterConstants.GoalType.LOW)
          .andThen(
              () -> RobotContainer.driveSubsystem.tankDriveVolts(Constants.zero, Constants.zero));

  private Command threeBallHighAuto =
      new ThreeBallSecondary(
              RobotContainer.driveSubsystem,
              RobotContainer.robotIntake,
              RobotContainer.fourBar,
              RobotContainer.shootSubsystem,
              RobotContainer.snekSystem,
              Constants.ShooterConstants.GoalType.HIGH)
          .andThen(
              () -> RobotContainer.driveSubsystem.tankDriveVolts(Constants.zero, Constants.zero));

  private Command twoBallHighAuto =
      new TwoBallSecondary(
              RobotContainer.driveSubsystem,
              RobotContainer.robotIntake,
              RobotContainer.fourBar,
              RobotContainer.shootSubsystem,
              RobotContainer.snekSystem,
              Constants.ShooterConstants.GoalType.HIGH)
          .andThen(
              () -> RobotContainer.driveSubsystem.tankDriveVolts(Constants.zero, Constants.zero));

  private Command twoBallLowAuto =
      new TwoBallSecondary(
              RobotContainer.driveSubsystem,
              RobotContainer.robotIntake,
              RobotContainer.fourBar,
              RobotContainer.shootSubsystem,
              RobotContainer.snekSystem,
              Constants.ShooterConstants.GoalType.LOW)
          .andThen(
              () -> RobotContainer.driveSubsystem.tankDriveVolts(Constants.zero, Constants.zero));

              
  private Command simpleScore =
      new SimpleScore(
          RobotContainer.driveSubsystem, RobotContainer.shootSubsystem, RobotContainer.snekSystem);

  private Command m_autonomousCommand;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();

    autoSelect.addOption("High 4 Ball Primary", fourBallHighAuto);
    autoSelect.addOption("Low 4 Ball Primary", fourBallLowAuto);
    autoSelect.addOption("High 2 Ball Secondary", twoBallHighAuto);
    autoSelect.addOption("Low 2 Ball Secondary", twoBallLowAuto);
    autoSelect.addOption("High 3 Ball Secondary", threeBallHighAuto);
    autoSelect.addOption("Low 3 Ball Secondary", threeBallLowAuto);

    autoSelect.addOption("Simple Score", simpleScore);

    SmartDashboard.putData("Auto Selector", autoSelect);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    if (DriverStation.isFMSAttached()) {
      RobotContainer.limelight.setLedMode(LedMode.PIPELINE);
    } else {
      RobotContainer.limelight.setLedMode(LedMode.FORCE_OFF);
    }
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    RobotContainer.limelight.setLedMode(LedMode.PIPELINE);

    m_autonomousCommand = autoSelect.getSelected();
    if (m_autonomousCommand == null) {
      m_autonomousCommand = fourBallLowAuto;
    }

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    RobotContainer.limelight.setLedMode(LedMode.PIPELINE);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // hello!
    // if (RobotContainer.snekSystem.getUpperLimit()) {
    //   RobotContainer.driver.setRumble(RumbleType.kLeftRumble, 0.5);
    //   RobotContainer.driver.setRumble(RumbleType.kRightRumble, 0.5);
    // } else {

    RobotContainer.driver.setRumble(RumbleType.kLeftRumble, 0);
    RobotContainer.driver.setRumble(RumbleType.kRightRumble, 0);
    // }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  public String goFast() {
    return "nyyooooom";
  }
}
