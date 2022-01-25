package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotMap;

public class IntakeFourBar extends SubsystemBase {

  private CANSparkMax fourBar;
  private double currPosition;
  public IntakeFourBar() {
    fourBar = new CANSparkMax(RobotMap.intakeMotorFourBar, MotorType.kBrushless);

    fourBar.restoreFactoryDefaults();

    fourBar.setSmartCurrentLimit(Constants.IntakeConstants.fourBarCurrentLimit);

    fourBar.setIdleMode(CANSparkMax.IdleMode.kBrake);
    fourBar.getPIDController().setP(Constants.IntakeConstants.kP);
  }

  public void setFourBarPosition(double position) {
    fourBar.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition);


  }

  @Override
  public void periodic() {

   SmartDashboard.putNumber("Four Bar Position", currPosition);

  }
}
