// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auto.FourBall;
import frc.robot.commands.auto.SimpleScore;
import frc.robot.commands.auto.TwoBallSecondary;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private SendableChooser<Command> autoSelect = new SendableChooser<>();
  private RobotContainer m_robotContainer = new RobotContainer();

  private Command fourBallAuto =
      new FourBall(
              RobotContainer.driveSubsystem,
              RobotContainer.robotIntake,
              RobotContainer.fourBar,
              RobotContainer.shootSubsystem,
              RobotContainer.snekSystem)
          .andThen(
              () -> RobotContainer.driveSubsystem.tankDriveVolts(Constants.zero, Constants.zero));

  private Command twoBallAuto =
      new TwoBallSecondary(
              RobotContainer.driveSubsystem,
              RobotContainer.robotIntake,
              RobotContainer.fourBar,
              RobotContainer.shootSubsystem,
              RobotContainer.snekSystem)
          .andThen(
              () -> RobotContainer.driveSubsystem.tankDriveVolts(Constants.zero, Constants.zero));

  private Command simpleScore =
      new SimpleScore(
          RobotContainer.driveSubsystem, RobotContainer.shootSubsystem, RobotContainer.snekSystem);

  private Command m_autonomousCommand =
      new FourBall(
              // new FourBall(
              RobotContainer.driveSubsystem,
              RobotContainer.robotIntake,
              RobotContainer.fourBar,
              RobotContainer.shootSubsystem,
              RobotContainer.snekSystem)
          .andThen(
              () -> RobotContainer.driveSubsystem.tankDriveVolts(Constants.zero, Constants.zero));
  // new SimpleScore(
  //     RobotContainer.driveSubsystem, RobotContainer.shootSubsystem, RobotContainer.snekSystem);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();
    autoSelect.setDefaultOption("4 ball (default)", fourBallAuto);
    autoSelect.addOption("4 ball", fourBallAuto);
    autoSelect.addOption("2 ball", twoBallAuto);
    autoSelect.addOption("simple score", simpleScore);
    SmartDashboard.putData(autoSelect);

    PortForwarder.add(5800, "limelight.local", 5800);
    PortForwarder.add(5801, "limelight.local", 5801);
    PortForwarder.add(5802, "limelight.local", 5802);
    PortForwarder.add(5803, "limelight.local", 5803);
    PortForwarder.add(5804, "limelight.local", 5804);
    PortForwarder.add(5805, "limelight.local", 5805);
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
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    // m_autonomousCommand = autoSelect.getSelected();
    m_autonomousCommand = fourBallAuto;

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
