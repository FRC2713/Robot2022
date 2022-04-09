// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class ShootWithLimelight extends CommandBase {
  ShootSubsystem shootSubsystem;
  LimelightSubsystem limelightSubsystem;
  /** Creates a new ShootWithLimelight. */
  public ShootWithLimelight(ShootSubsystem shootSubsystem, LimelightSubsystem limelightSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shootSubsystem = shootSubsystem;
    this.limelightSubsystem = limelightSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shootSubsystem.shootAtDistance(limelightSubsystem.getVerticalOffset());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shootSubsystem.topCloseEnough() && shootSubsystem.primaryCloseEnough();
  }
}
