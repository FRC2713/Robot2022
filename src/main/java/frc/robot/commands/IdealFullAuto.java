package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeFourBar;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class IdealFullAuto extends SequentialCommandGroup {
  Trajectory leg1 =
      PathPlanner.loadPath(
          "Leg1", Constants.AutoConstants.maxSpeed, Constants.AutoConstants.maxAccel);
  Trajectory leg2 =
      PathPlanner.loadPath(
          "Leg2", Constants.AutoConstants.maxSpeed, Constants.AutoConstants.maxAccel);

  public IdealFullAuto(ShootSubsystem shootSubsystem, DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, IntakeFourBar intakeFourBar) {
    addCommands(
        new DeployIntake(intakeSubsystem, intakeFourBar), //deploys intake
        //command that makes the snek schmoove
        new RamsetA(leg1, driveSubsystem), //first leg of the journey, ideally picks up a ball on the way
        new ShootALowBall(shootSubsystem), //shoots until the ball goes by, in theory; will need two of these eventually for two balls
        new RamsetA(leg2, driveSubsystem), //second leg of the journey, should pick up two balls on the way
        new ShootALowBall(shootSubsystem), //shoots until the ball goes by, in theory; will need two of these eventually for two balls
        new IntakeSetRollers(intakeSubsystem, 0) //turns off the rollers
        );
  }
}
