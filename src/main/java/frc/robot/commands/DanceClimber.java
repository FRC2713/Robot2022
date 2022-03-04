package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class DanceClimber extends CommandBase {
     private final ClimberSubsystem climb;

     public DanceClimber (ClimberSubsystem climb)
     {
         this.climb = climb ;
         addRequirements(climb);
     }
@Override
    public void initialize ()
{

}
@Override
    public void execute ()
{

}
//@Override
   // public void end (boolean )
}

