package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotMap;

public class ClimberSubsystem extends SubsystemBase {

    private final CANSparkMax telescope;

    public ClimberSubsystem (){

        telescope = new CANSparkMax(RobotMap.telescope, MotorType.kBrushless);

        telescope.restoreFactoryDefaults(); //Seems like an important thing

        telescope.setIdleMode(IdleMode.kBrake/*Might change idk what we want*/);

        telescope.setInverted(true/*I think thats how gear stuff shall require it*/);
    }

    public void setTelescopeSpeed(double speed){
        telescope.set(speed);
    }


}
