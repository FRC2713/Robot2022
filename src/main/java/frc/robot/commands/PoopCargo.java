// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.SnekConstants;
import frc.robot.subsystems.IntakeFourBar;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SnekSystem;

public class PoopCargo extends CommandBase {
  /** Creates a new AutoEmptySnek. */
  Debouncer completelyEmpty = new Debouncer(SnekConstants.debouncerDuration);

  Timer timer;

  SnekSystem snekSystem;
  IntakeSubsystem intakeSubsystem;
  IntakeFourBar intakeFourBar;

  public PoopCargo(
      SnekSystem snekSystem, IntakeSubsystem intakeSubsystem, IntakeFourBar intakeFourBar) {
    this.snekSystem = snekSystem;
    this.intakeSubsystem = intakeSubsystem;
    this.intakeFourBar = intakeFourBar;
    addRequirements(snekSystem, intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    snekSystem.setUpperSnekSpeed(-1.0);
    snekSystem.setLowerSnekSpeed(-1.0);
    new IntakeExtendToLimit(intakeFourBar, Constants.IntakeConstants.intakeExtensionSpeed);
    new IntakeSetRollers(intakeSubsystem, -Constants.IntakeConstants.typicalRollerRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    snekSystem.setLowerSnekSpeed(0.0);
    snekSystem.setUpperSnekSpeed(0.0);
    new IntakeExtendToLimit(intakeFourBar, -Constants.IntakeConstants.intakeExtensionSpeed);
    new IntakeSetRollers(intakeSubsystem, Constants.zero);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return completelyEmpty.calculate(!snekSystem.getLowerLimit() && !snekSystem.getUpperLimit());
  }
}
