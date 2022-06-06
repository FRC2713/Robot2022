// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.SnekConstants;
import frc.robot.subsystems.IntakeFourBar;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SnekSystem;

public class PoopCargo extends SequentialCommandGroup {
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
    addCommands(
        new IntakeExtendToLimit(intakeFourBar, Constants.IntakeConstants.intakeExtensionSpeed),
        new IntakeSetRollers(intakeSubsystem, -Constants.IntakeConstants.typicalRollerRPM),
        new RunCommand(
                () -> {
                  snekSystem.setUpperSnekSpeed(-1.0);
                  snekSystem.setLowerSnekSpeed(-1.0);
                },
                snekSystem)
            .perpetually()
            .withInterrupt(
                () ->
                    completelyEmpty.calculate(
                        !snekSystem.getLowerLimit() && !snekSystem.getUpperLimit())),
        new InstantCommand(
            () -> {
              snekSystem.setLowerSnekSpeed(0.0);
              snekSystem.setUpperSnekSpeed(0.0);
            },
            snekSystem),
        new IntakeExtendToLimit(intakeFourBar, -Constants.IntakeConstants.intakeExtensionSpeed),
        new IntakeSetRollers(intakeSubsystem, Constants.zero));
  }
}
