// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Hello World

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auto.TwoBallSecondary;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private RobotContainer m_robotContainer = new RobotContainer(); // DO NOT DELETE THIS

  private SendableChooser<Command> autoSelect = new SendableChooser<>();

  // private Command fourBall =
  //     new FourBall(
  //             RobotContainer.driveSubsystem,
  //             RobotContainer.robotIntake,
  //             RobotContainer.fourBar,
  //             RobotContainer.shootSubsystem,
  //             RobotContainer.snekSystem,
  //             RobotContainer.limelight,
  //             RobotContainer.strip)
  //         .andThen(
  //             () -> RobotContainer.driveSubsystem.tankDriveVolts(Constants.zero,
  // Constants.zero));

  // private Command threeBallHighAuto =
  //     new ThreeBallSecondary(
  //             RobotContainer.driveSubsystem,
  //             RobotContainer.robotIntake,
  //             RobotContainer.fourBar,
  //             RobotContainer.shootSubsystem,
  //             RobotContainer.snekSystem,
  //             RobotContainer.limelight,
  //             RobotContainer.strip)
  //         .andThen(
  //             () -> RobotContainer.driveSubsystem.tankDriveVolts(Constants.zero,
  // Constants.zero));

  // private Command threeBallPartner =
  //     new ThreeBallPartnerSecondary(
  //         RobotContainer.driveSubsystem,
  //         RobotContainer.robotIntake,
  //         RobotContainer.fourBar,
  //         RobotContainer.shootSubsystem,
  //         RobotContainer.snekSystem,
  //         RobotContainer.limelight,
  //         RobotContainer.strip);

  // private Command twoBall =
  //     new TwoBallSecondary(
  //             RobotContainer.driveSubsystem,
  //             RobotContainer.robotIntake,
  //             RobotContainer.fourBar,
  //             RobotContainer.shootSubsystem,
  //             RobotContainer.snekSystem,
  //             RobotContainer.limelight,
  //             RobotContainer.strip)
  //         .andThen(
  //             () -> RobotContainer.driveSubsystem.tankDriveVolts(Constants.zero,
  // Constants.zero));

  // private Command fiveBall =
  //     new FiveBall(
  //             RobotContainer.driveSubsystem,
  //             RobotContainer.robotIntake,
  //             RobotContainer.fourBar,
  //             RobotContainer.shootSubsystem,
  //             RobotContainer.snekSystem,
  //             RobotContainer.limelight,
  //             RobotContainer.strip)
  //         .andThen(
  //             () -> RobotContainer.driveSubsystem.tankDriveVolts(Constants.zero,
  // Constants.zero));

  // private Command simpleScore =
  //     new SimpleScore(
  //         RobotContainer.driveSubsystem, RobotContainer.shootSubsystem,
  // RobotContainer.snekSystem);

  private Command m_autonomousCommand;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();

    // autoSelect.addOption("Five Ball", fiveBall);
    // autoSelect.addOption("Four Ball", fourBall);
    // autoSelect.addOption("Two Ball Secondary", twoBall);
    // autoSelect.addOption("Three Ball Secondary", threeBallHighAuto);
    // autoSelect.addOption("Three Ball Partner Secondary", threeBallPartner);

    // autoSelect.addOption("Simple Score", simpleScore);

    SmartDashboard.putData("Auto Selector", autoSelect);

    Field2d field = new Field2d();
    field.getObject("reference").setTrajectory(TwoBallSecondary.leg2);
    SmartDashboard.putData(field);
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
    // if (DriverStation.isFMSAttached()) {
    // RobotContainer.limelight.setLedMode(LedMode.FORCE_ON);
    // } else {
    // RobotContainer.limelight.setLedMode(LedMode.FORCE_OFF);
    // }
    // RobotContainer.driveSubsystem.setAllCoast();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // RobotContainer.limelight.setLedMode(LedMode.FORCE_ON);

    // m_autonomousCommand = autoSelect.getSelected();
    // if (m_autonomousCommand == null) {
    //   m_autonomousCommand = fourBall;
    // }

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
    // RobotContainer.limelight.setLedMode(LedMode.FORCE_ON);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    RobotContainer.shootSubsystem.setPrimaryRPM(0);
    RobotContainer.shootSubsystem.setTopRPM(0);
    // RobotContainer.driveSubsystem.setHalfBrakeHalfCoast();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // hello!
    // if (RobotContainer.snekSystem.getUpperLimit()) {
    //   RobotContainer.driver.setRumble(RumbleType.kLeftRumble, 0.5);
    //   RobotContainer.driver.setRumble(RumbleType.kRightRumble, 0.5);
    // } else {

    // RobotContainer.driver.setRumble(RumbleType.kLeftRumble, 0);
    // RobotContainer.driver.setRumble(RumbleType.kRightRumble, 0);
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
