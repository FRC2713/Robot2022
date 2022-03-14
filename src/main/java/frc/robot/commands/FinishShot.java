// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SnekSystem;

public class FinishShot extends CommandBase {
  /** Creates a new AutoEmptySnek. */
  Debouncer completelyEmpty = new Debouncer(0.5);

  SnekSystem snekSystem;

  public FinishShot(SnekSystem snekSystem) {
    this.snekSystem = snekSystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new SetSnekSpeed(snekSystem, 1, 1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    new SetSnekSpeed(snekSystem, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return completelyEmpty.calculate(!snekSystem.getLowerLimit() && !snekSystem.getUpperLimit());
  }
}
