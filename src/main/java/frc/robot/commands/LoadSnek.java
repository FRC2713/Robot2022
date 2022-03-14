// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SnekSystem;

public class LoadSnek extends CommandBase {
  /** Creates a new loadSnek. */
  private final SnekSystem snekSystem;

  public LoadSnek(SnekSystem snekSystem) {
    this.snekSystem = snekSystem;
    addRequirements(snekSystem);
  }

  @Override
  public void execute() {
    snekSystem.loadSnek();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
