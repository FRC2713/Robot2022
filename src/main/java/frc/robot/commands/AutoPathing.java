package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoPathing extends SequentialCommandGroup {
  Trajectory leg1 =
      PathPlanner.loadPath(
          "Leg1", Constants.AutoConstants.maxSpeed, Constants.AutoConstants.maxAccel);
  Trajectory leg2 =
      PathPlanner.loadPath(
          "Leg2", Constants.AutoConstants.maxSpeed, Constants.AutoConstants.maxAccel);

  public AutoPathing(DriveSubsystem driveSubsystem) {

    addCommands(
        RamsetA.RamseteSchmoove(leg1, driveSubsystem),
        new WaitCommand(1));
//        new RamsetA().RamseteSchmoove(leg2, driveSubsystem),
//        new WaitCommand(1));
  }
}
