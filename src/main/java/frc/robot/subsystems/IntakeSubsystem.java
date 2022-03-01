/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotMap;

public class IntakeSubsystem extends SubsystemBase {

  private CANSparkMax rollers;

  public IntakeSubsystem() {
    rollers = new CANSparkMax(RobotMap.intakeMotorRollers, MotorType.kBrushless);

    rollers.restoreFactoryDefaults();

    rollers.setSmartCurrentLimit(Constants.IntakeConstants.rollerCurrentLimit);
    rollers.setIdleMode(IdleMode.kCoast);
    rollers.getEncoder().setVelocityConversionFactor(Constants.IntakeConstants.rollerRatio / 60);
  }

  public void setRollerRPM(double rpm) {
    rollers.set(rpm / Constants.IntakeConstants.maxRollerRpm);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Roller Speed", rollers.getEncoder().getVelocity());
  }
}
