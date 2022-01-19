// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {}

  private CANSparkMax frontOne =
      new CANSparkMax(
          Constants.RobotMap.frontOneMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax frontTwo =
      new CANSparkMax(
          Constants.RobotMap.frontTwoMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax backOne =
      new CANSparkMax(
          Constants.RobotMap.backOneMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax backTwo =
      new CANSparkMax(
          Constants.RobotMap.backTwoMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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

    frontOne.set(right);
    frontTwo.set(left);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
