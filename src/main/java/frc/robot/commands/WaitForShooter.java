// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShootSubsystem;

public class WaitForShooter extends CommandBase {
  ShootSubsystem shootSystem;
  Debouncer isCloseEnough;
  /** Creates a new WaitForShooter. */
  public WaitForShooter(ShootSubsystem shootSubsystem) {
    this.shootSystem = shootSubsystem;
    isCloseEnough = new Debouncer(0.1);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isCloseEnough.calculate(
        this.shootSystem.primaryCloseEnough() && this.shootSystem.topCloseEnough());
  }
}
