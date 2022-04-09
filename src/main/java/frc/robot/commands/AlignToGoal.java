// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.StripSubsystem;
import frc.robot.subsystems.StripSubsystem.Pattern;

public class AlignToGoal extends CommandBase {

  SimpleMotorFeedforward feedForward =
      new SimpleMotorFeedforward(
          // Constants.AutoConstants.ksVolts,
          Constants.LimelightConstants.kTurnInPlaceStaticVolts.get(),
          Constants.AutoConstants.kvVoltSecondsPerMeter,
          Constants.AutoConstants.kaVoltSecondsSquaredPerMeter);

  PIDController leftController = new PIDController(Constants.AutoConstants.kPDriveVel, 0, 0);
  PIDController rightController = new PIDController(Constants.AutoConstants.kPDriveVel, 0, 0);
  PIDController rotatController =
      new PIDController(Constants.LimelightConstants.rotationKP.get(), 0, 0);

  Debouncer onTargetDebouncer;

  DriveSubsystem driveSubsystem;
  LimelightSubsystem limelightSubsystem;
  StripSubsystem stripSubsystem;
  /** Creates a new AlignToGoal. */
  public AlignToGoal(
      DriveSubsystem driveSubsystem,
      LimelightSubsystem limelightSubsystem,
      StripSubsystem stripSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    this.stripSubsystem = stripSubsystem;
    this.onTargetDebouncer = new Debouncer(.25);

    addRequirements(driveSubsystem, limelightSubsystem, stripSubsystem);

    rotatController.setTolerance(Constants.LimelightConstants.rotationalTolerance.get());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Constants.tuningMode) {
      rotatController = new PIDController(Constants.LimelightConstants.rotationKP.get(), 0, 0);
      rotatController.setTolerance(Constants.LimelightConstants.rotationalTolerance.get());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!limelightSubsystem.hasValidTargets()) {
      stripSubsystem.setColor(Pattern.StrobeRed);
      return;
    } else {
      stripSubsystem.setColor(Pattern.White);
    }

    double error = limelightSubsystem.getHorizontalOffset();
    double targetWheelSpeed = rotatController.calculate(error, 0);
    double leftOutput =
        leftController.calculate(
                driveSubsystem.getWheelSpeeds().leftMetersPerSecond, -targetWheelSpeed)
            + feedForward.calculate(-targetWheelSpeed);
    double rightOutput =
        rightController.calculate(
                driveSubsystem.getWheelSpeeds().rightMetersPerSecond, targetWheelSpeed)
            + feedForward.calculate(targetWheelSpeed);

    SmartDashboard.putNumber("AlignLeft", leftOutput);
    SmartDashboard.putNumber("AlignRight", rightOutput);

    driveSubsystem.tankDriveVolts(leftOutput, rightOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.tankDriveVolts(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return onTargetDebouncer.calculate(rotatController.atSetpoint());
  }
}
