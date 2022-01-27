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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotMap;

public class IntakeSubsystem extends SubsystemBase {

  private CANSparkMax rollers;
  DigitalInput bottomSwitch;
  DigitalInput topSwitch; 


  public IntakeSubsystem() {
    rollers = new CANSparkMax(RobotMap.intakeMotorPrimary, MotorType.kBrushless);

    rollers.restoreFactoryDefaults();

    rollers.setSmartCurrentLimit(Constants.IntakeConstants.rollerCurrentLimit);

    rollers.setIdleMode(IdleMode.kCoast);

    bottomSwitch = new DigitalInput(RobotMap.intakeBottomSwitch);

    topSwitch = new DigitalInput(RobotMap.intakeTopSwitch);

  }

  public void setRollerSpeed(double speed) {
    rollers.set(speed);
  }

  
  @Override
  public void periodic() {

  }

}