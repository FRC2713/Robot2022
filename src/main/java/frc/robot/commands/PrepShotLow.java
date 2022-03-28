// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.SnekSystem;

public class PrepShotLow extends SequentialCommandGroup {

  /** Creates a new PrepShot. */
  public PrepShotLow(ShootSubsystem shootSubsystem, SnekSystem snekSystem, boolean shouldRollback) {
    if (shouldRollback) {
      addCommands(
          // new SetSnekSpeed(
          // snekSystem,
          // Constants.SnekConstants.upperReversePower,
          // Constants.SnekConstants.lowerReversePower)
          // .perpetually()
          // .withTimeout(Constants.SnekConstants.reverseDuration),
          new ParallelCommandGroup(
              new SetSnekSpeed(snekSystem, 0, 0)
                  .withInterrupt(
                      () ->
                          (shootSubsystem.primaryCloseEnough() && shootSubsystem.topCloseEnough())),
              new SetShooterRPM(
                  shootSubsystem,
                  Constants.ShooterConstants.primaryLowShotSpeed.get(),
                  Constants.ShooterConstants.topLowShotSpeed.get(),
                  Constants.ShooterConstants.waitUntilAtSpeed)));
    } else {
      addCommands(
          // new SetSnekSpeed(snekSystem, 0, 0)
          //     .withInterrupt(
          //         () -> (shootSubsystem.primaryCloseEnough() &&
          // shootSubsystem.topCloseEnough())),
          new SetShooterRPM(
              shootSubsystem,
              Constants.ShooterConstants.primaryLowShotSpeed.get(),
              Constants.ShooterConstants.topLowShotSpeed.get(),
              Constants.ShooterConstants.waitUntilAtSpeed));
    }
  }
}
