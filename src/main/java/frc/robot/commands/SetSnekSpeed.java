// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SnekSystem;

public class SetSnekSpeed extends CommandBase {
  private final SnekSystem snekSystem;
  private final double upper;
  private final double lower;

  /** Creates a new SetSnekSpeed. */
  public SetSnekSpeed(SnekSystem snekSystem, double upper, double lower) {
    this.snekSystem = snekSystem;
    this.upper = upper;
    this.lower = lower;
    addRequirements(snekSystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    snekSystem.setLowerSnekSpeed(lower);
    snekSystem.setUpperSnekSpeed(upper);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
