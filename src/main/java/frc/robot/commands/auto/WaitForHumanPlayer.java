// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SnekSystem;

public class WaitForHumanPlayer extends WaitCommand {
  SnekSystem snekSystem;

  DriveSubsystem driveSubsystem;

  public WaitForHumanPlayer(double duration, SnekSystem snekSystem, DriveSubsystem driveSubsystem) {
    super(duration);
    this.snekSystem = snekSystem;
    this.driveSubsystem = driveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.tankDriveVolts(
        AutoConstants.crawlTowardsHumanPlayerVolts, AutoConstants.crawlTowardsHumanPlayerVolts);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    snekSystem.loadSnek();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.tankDriveVolts(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (snekSystem.getLowerLimit() && snekSystem.getUpperLimit()) || super.isFinished();
  }
}
