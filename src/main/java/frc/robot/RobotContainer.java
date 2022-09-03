// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SnekConstants;
import frc.robot.commands.AlignToGoal;
import frc.robot.commands.ClimberSetHeight;
import frc.robot.commands.FeedWithDelay;
import frc.robot.commands.PoopCargo;
import frc.robot.commands.PrepShotHigh;
import frc.robot.commands.PrepShotLow;
import frc.robot.commands.SetShooterRPM;
import frc.robot.commands.SetSnekSpeed;
import frc.robot.commands.groups.IntakePreventThreeBallActive;
import frc.robot.commands.groups.IntakePreventThreeBallInactive;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeFourBar;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.SnekSystem;
import frc.robot.subsystems.StripSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.StripSubsystem.Pattern;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public static final IntakeSubsystem robotIntake = new IntakeSubsystem();
  public static final IntakeFourBar fourBar = new IntakeFourBar();
  public static final ShootSubsystem shootSubsystem = new ShootSubsystem();
  public static final SnekSystem snekSystem = new SnekSystem();
  public static final StripSubsystem strip = new StripSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();
  public static final LimelightSubsystem limelight = new LimelightSubsystem();

  public static final XboxController driver = new XboxController(Constants.zero);
  public static final XboxController operator = new XboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    swerveSubsystem.setDefaultCommand(
        new RunCommand(
            () -> {
              swerveSubsystem.drive(
                  driver.getLeftY() * Constants.DriveConstants.maxSwerveVel, driver.getLeftX() * Constants.DriveConstants.maxSwerveVel, driver.getRightX() * Constants.DriveConstants.maxSwerveAzi);
            },
            swerveSubsystem));

    // driveSubsystem.setDefaultCommand(
    //     new RunCommand(
    //         () -> {
    //           driveSubsystem.tankDriveVolts(
    //               -LimelightConstants.kTurnInPlaceStaticVolts.get(),
    //               LimelightConstants.kTurnInPlaceStaticVolts.get());
    //         },
    //         driveSubsystem));

    climber.setDefaultCommand(
        new RunCommand(
            () -> {
              // climber.setTelescopeSpeed(-MathUtil.applyDeadband(operator.getRightY(), 0.1));
              double speedOfChange = -MathUtil.applyDeadband(operator.getRightY(), 0.1) * 120 * 0.3;
              climber.setTargetHeight(climber.getTargetHeight() + speedOfChange * 0.02);
            },
            climber));

    snekSystem.setDefaultCommand(
        new RunCommand(
            () -> {
              snekSystem.loadSnek();
            },
            snekSystem));

    strip.setDefaultCommand(
        new RunCommand(
            () -> {
              if (Constants.ClimberConstants.midHeight + 2 >= climber.getLeftHeight()
                  && climber.getLeftHeight() >= Constants.ClimberConstants.midHeight - 2) {
                strip.setColor(Pattern.Green);
              } else if (snekSystem.getUpperLimit() && snekSystem.getLowerLimit()) {
                strip.setColor(Pattern.White);
              } else if (snekSystem.getUpperLimit() || snekSystem.getLowerLimit()) {
                strip.setColor(Pattern.Yellow);
              } else {
                strip.setAllianceColor(strip);
              }
            },
            strip));

    // shootSubsystem.setDefaultCommand(new RunCommand(() -> {
    //   if (snekSystem.getLowerLimit() || snekSystem.getUpperLimit()) {
    //     if (limelight.hasValidTargets()) {
    //       shootSubsystem.shootAtDistance(limelight.getHorizontalOffset());
    //     } else {
    //       shootSubsystem.setPrimaryRPM(ShooterConstants.primaryLowShotSpeed.get());
    //       shootSubsystem.setTopRPM(ShooterConstants.topLowShotSpeed.get());
    //     }
    //   }
    // }, shootSubsystem));

    // shootSubsystem.setDefaultCommand(new RunCommand(() -> {

    // }, shootSubsystem));

    new Trigger(
            () ->
                snekSystem.getUpperLimit()
                    && !snekSystem.getLowerLimit()
                    && shootSubsystem.getPrimarySpeed() < 10)
        .whenActive(
            new SequentialCommandGroup(
                new SetSnekSpeed(snekSystem, -0.03, 0).withTimeout(0.10),
                new SetShooterRPM(
                    shootSubsystem,
                    ShooterConstants.primaryLowShotSpeed.get(),
                    ShooterConstants.topLowShotSpeed.get(),
                    true)));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(driver, XboxController.Button.kLeftBumper.value)
        .whileActiveOnce(
            new SequentialCommandGroup(
                new PrepShotLow(shootSubsystem, snekSystem, true),
                new SetSnekSpeed(snekSystem, 1.0, 1.0).perpetually()))
        .whenInactive(
            new ParallelCommandGroup(
                new SetSnekSpeed(snekSystem, Constants.zero, Constants.zero),
                new SetShooterRPM(shootSubsystem, Constants.zero, Constants.zero, false)));

    // new JoystickButton(driver, XboxController.Button.kX.value)
    //     .whileActiveOnce(
    //         new SequentialCommandGroup(
    //             // new PrepShotHigh(shootSubsystem, snekSystem, true),
    //             new SetShooterSpin(shootSubsystem, 8 /* m/s */, 0 /* spin */, true),
    //             new FeedWithDelay(snekSystem, SnekConstants.secondHighShotDelay)
    //             // new FeedWithSmartDelay(
    //             //     snekSystem, shootSubsystem, SnekConstants.secondHighShotDelay + 3)
    //             // new SetSnekSpeed(snekSystem, 0.6, 0.6).perpetually()
    //             ))
    //     .whenInactive(
    //         new ParallelCommandGroup(
    //             new SetSnekSpeed(snekSystem, Constants.zero, Constants.zero),
    //             new SetShooterRPM(shootSubsystem, Constants.zero, Constants.zero, false)));

    // new JoystickButton(driver, XboxController.Button.kX.value)
    //     .whenHeld(new SetSnekSpeed(snekSystem, 1.0, 1.0));

    new JoystickButton(driver, XboxController.Button.kX.value)
        .whileActiveContinuous(new SetSnekSpeed(snekSystem, 1.0, 1.0))
        .whenInactive(new SetSnekSpeed(snekSystem, Constants.zero, Constants.zero));

    new JoystickButton(driver, XboxController.Button.kRightBumper.value)
        .whileActiveOnce(
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new PrepShotHigh(shootSubsystem, snekSystem, limelight, true),
                        new AlignToGoal(swerveSubsystem, limelight, strip)),
                    new FeedWithDelay(snekSystem, SnekConstants.secondHighShotDelay)
                    // new FeedWithSmartDelay(
                    //     snekSystem, shootSubsystem, SnekConstants.secondHighShotDelay + 3)
                    // new SetSnekSpeed(snekSystem, 0.6, 0.6).perpetually()
                    )))
        .whenInactive(
            new ParallelCommandGroup(
                new SetSnekSpeed(snekSystem, Constants.zero, Constants.zero),
                new SetShooterRPM(shootSubsystem, Constants.zero, Constants.zero, false)));

    // new JoystickButton(driver, XboxController.Button.kLeftBumper.value)
    //     .whileActiveContinuous(new SetSnekSpeed(snekSystem, 0.75, 0.75))
    //     .whenInactive(new SetSnekSpeed(snekSystem, 0, 0));

    // new JoystickButton(driver, XboxController.Button.kLeftBumper.value)
    //     .whileHeld(
    //         new ParallelCommandGroup(
    //             new SetSnekSpeed(snekSystem, 1.0, 1.0),
    //             new IntakeExtendToLimit(
    //                 fourBar, Constants.IntakeConstants.intakeExtensionSpeed / 2),
    //             new IntakeSetRollers(robotIntake, Constants.IntakeConstants.typicalRollerRPM)))
    //     .whenInactive(
    //         new ParallelCommandGroup(
    //             new SetSnekSpeed(snekSystem, 0, 0),
    //             new IntakeSetRollers(robotIntake, Constants.zero)));

    new Trigger(() -> (operator.getBackButton() && operator.getStartButton()))
        .whenActive(
            new InstantCommand(
                () -> {
                  fourBar.setOperatorControlled(!fourBar.getOperatorControlled());
                },
                fourBar));

    new JoystickButton(driver, XboxController.Button.kY.value)
        .whileActiveOnce(new IntakePreventThreeBallActive(robotIntake, snekSystem, fourBar))
        .whenInactive(new IntakePreventThreeBallInactive(robotIntake, snekSystem, fourBar));

    new JoystickButton(driver, XboxController.Button.kB.value)
        .whileActiveOnce(new PoopCargo(snekSystem, robotIntake, fourBar))
        .whenInactive(new IntakePreventThreeBallInactive(robotIntake, snekSystem, fourBar));

    new JoystickButton(operator, XboxController.Button.kX.value)
        .whenPressed(new ClimberSetHeight(climber, Constants.ClimberConstants.lowHeight));

    new JoystickButton(operator, XboxController.Button.kA.value)
        .whenPressed(new ClimberSetHeight(climber, Constants.ClimberConstants.minimumHeight));

    new JoystickButton(operator, XboxController.Button.kY.value)
        .whenPressed(new ClimberSetHeight(climber, Constants.ClimberConstants.maximumHeight));
  }
}
