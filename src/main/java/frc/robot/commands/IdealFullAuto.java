package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class IdealFullAuto extends SequentialCommandGroup {
  Trajectory leg1 =
      PathPlanner.loadPath(
          "Leg1", Constants.AutoConstants.maxSpeed, Constants.AutoConstants.maxAccel);
  Trajectory leg2 =
      PathPlanner.loadPath(
          "Leg2", Constants.AutoConstants.maxSpeed, Constants.AutoConstants.maxAccel);

  public IdealFullAuto(ShootSubsystem shootSubsystem, DriveSubsystem driveSubsystem) {
    addCommands(
        // intake to full speed
        new RamsetA(leg1, driveSubsystem),
        new ShootALowBall(shootSubsystem),
        new RamsetA(leg2, driveSubsystem),
        new ShootALowBall(shootSubsystem)
        // intake back to 0
        );
  }
}
