package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShootSubsystem extends SubsystemBase {
    private double kp = 0.9;
    private CANSparkMax fly1 = new CANSparkMax(Constants.RobotMap.flywheelLeftPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax fly2 = new CANSparkMax(Constants.RobotMap.flywheelRightPort, CANSparkMaxLowLevel.MotorType.kBrushless);

    public ShootSubsystem() {
        fly2.follow(fly1);
    }

    public void setTargetRPM (int targetRPM) {
        //PID stuff :)
        fly1.getPIDController().setReference(targetRPM, ControlType.kVelocity);
    }

    public void updatePID () {
    }
}