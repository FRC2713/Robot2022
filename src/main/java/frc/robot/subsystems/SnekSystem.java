package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SnekSystem extends SubsystemBase {
    private CANSparkMax lowerSnek = new CANSparkMax(Constants.RobotMap.lowerSnek, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax upperSnek = new CANSparkMax(Constants.RobotMap.upperSnek, CANSparkMaxLowLevel.MotorType.kBrushless);
    private DigitalInput lowerLimit = new DigitalInput(Constants.RobotMap.lowerSnek);
    private DigitalInput upperLimit = new DigitalInput(Constants.RobotMap.upperSnek);
    


    public SnekSystem() {
        //ya mum
        lowerSnek.restoreFactoryDefaults();
        upperSnek.restoreFactoryDefaults();

        lowerSnek.setSmartCurrentLimit(Constants.SnekConstants.currentLimit);
        upperSnek.setSmartCurrentLimit(Constants.SnekConstants.currentLimit);
    }

    public void loadSnek() {
        if(lowerLimit.get() && upperLimit.get()) {
            lowerSnek.set(0);
            upperSnek.set(0);
        }
        else if (!lowerLimit.get() && upperLimit.get()) {
            lowerSnek.set(0.5);
            upperSnek.set(0);
        }
        else {
            lowerSnek.set(0.5);
            upperSnek.set(0.5);
        }
    }

    public void setLowerSnekSpeed(double speed) {
        lowerSnek.set(speed);
    }

    public void setUpperSnekSpeed(double speed) {
        upperSnek.set(speed);
    }
}
