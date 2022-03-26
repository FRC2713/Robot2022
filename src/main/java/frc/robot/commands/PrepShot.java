// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.SnekSystem;

public class PrepShot extends SequentialCommandGroup {

  /** Creates a new PrepShot. */
  public PrepShot(ShootSubsystem shootSubsystem, SnekSystem snekSystem, boolean shouldRollback) {
    if (shouldRollback) {
      addCommands(
          // new SetSnekSpeed(
          //         snekSystem,
          //         Constants.SnekConstants.upperReversePower,
          //         Constants.SnekConstants.lowerReversePower)
          //     .perpetually()
          //     .withTimeout(Constants.SnekConstants.reverseDuration),
          new ParallelCommandGroup(
              new SetSnekSpeed(
                      snekSystem,
                      Constants.SnekConstants.upperReversePower,
                      Constants.SnekConstants.lowerReversePower)
                  .perpetually()
                  .withInterrupt(shootSubsystem::closeEnough),
              new SetShooterRPM(
                  shootSubsystem,
                  Constants.ShooterConstants.typicalShotSpeed.get(),
                  Constants.ShooterConstants.waitUntilAtSpeed)));
    } else {
      addCommands(
          // new SetSnekSpeed(snekSystem, 0, 0).withInterrupt(() ->
          // shootSubsystem.closeEnough()),
          new SetShooterRPM(
              shootSubsystem,
              Constants.ShooterConstants.typicalShotSpeed.get(),
              Constants.ShooterConstants.waitUntilAtSpeed));
    }
  }
}
