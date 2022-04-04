// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SnekSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FeedWithDelay extends SequentialCommandGroup {
  public FeedWithDelay(SnekSystem snekSystem, double delaySeconds) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SetSnekSpeed(snekSystem, 1.0, 0.0),
        new WaitCommand(delaySeconds),
        new SetSnekSpeed(snekSystem, 1.0, 1.0).perpetually());
  }
}
