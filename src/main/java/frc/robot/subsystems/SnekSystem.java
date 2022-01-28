package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SnekSystem extends SubsystemBase {
    private CANSparkMax lowerSnek = new CANSparkMax(Constants.RobotMap.lowerSnek, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax upperSnek = new CANSparkMax(Constants.RobotMap.upperSnek, CANSparkMaxLowLevel.MotorType.kBrushless);
    


    public SnekSystem() {
        //ya mum
    }

    public void setLowerSnekSpeed(double speed) {
        lowerSnek.set(speed);
    }

    public void setUpperSnekSpeed(double speed) {
        upperSnek.set(speed);
    }
}
