// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.StripSubsystem;

public abstract class TurnInPlace extends CommandBase {

  SimpleMotorFeedforward feedForward =
      new SimpleMotorFeedforward(
          // Constants.AutoConstants.ksVolts,
          Constants.LimelightConstants.kTurnInPlaceStaticVolts.get(),
          // Constants.AutoConstants.kvVoltSecondsPerMeter,
          // Constants.AutoConstants.kaVoltSecondsSquaredPerMeter);
          0,
          0);

  PIDController leftController = new PIDController(Constants.AutoConstants.kPDriveVel, 0, 0);
  PIDController rightController = new PIDController(Constants.AutoConstants.kPDriveVel, 0, 0);
  PIDController rotatController =
      new PIDController(Constants.LimelightConstants.rotationKP.get(), 0, 0);

  Debouncer onTargetDebouncer;

  DriveSubsystem driveSubsystem;
  StripSubsystem stripSubsystem;
  double setpoint;

  public TurnInPlace(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.onTargetDebouncer = new Debouncer(.125);

    addRequirements(driveSubsystem);

    rotatController.setTolerance(Constants.LimelightConstants.rotationalTolerance.get());
    rotatController.enableContinuousInput(0, 360);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Constants.tuningMode) {
      rotatController = new PIDController(Constants.LimelightConstants.rotationKP.get(), 0, 0);
      rotatController.setTolerance(Constants.LimelightConstants.rotationalTolerance.get());
      rotatController.enableContinuousInput(-180, 180);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = (getSetpoint() - getMeasurement());
    SmartDashboard.putNumber("AlignError", error);

    double targetWheelSpeed = rotatController.calculate(getMeasurement(), getSetpoint());
    double leftOutput =
        leftController.calculate(
                driveSubsystem.getWheelSpeeds().leftMetersPerSecond, -targetWheelSpeed)
            + feedForward.calculate(-targetWheelSpeed)
            - (Constants.LimelightConstants.kTurnInPlaceStaticVolts.get());
    double rightOutput =
        rightController.calculate(
                driveSubsystem.getWheelSpeeds().rightMetersPerSecond, targetWheelSpeed)
            + feedForward.calculate(targetWheelSpeed)
            + Constants.LimelightConstants.kTurnInPlaceStaticVolts.get();

    double voltageLimit = 4;
    leftOutput = MathUtil.clamp(leftOutput, -voltageLimit, voltageLimit);
    rightOutput = MathUtil.clamp(rightOutput, -voltageLimit, voltageLimit);

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
    SmartDashboard.putNumber("AlignSetpoint", rotatController.getSetpoint());
    return onTargetDebouncer.calculate(rotatController.atSetpoint());
  }

  public abstract double getMeasurement();

  public abstract double getSetpoint();
}
