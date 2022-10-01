package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants;

public class OffsetAbsoluteAnalogEncoder {

    private final double VOLTAGE_OFFSET;

    private AnalogInput analogInput;

    public OffsetAbsoluteAnalogEncoder(int port, double voltageOffset) {
        this.analogInput = new AnalogInput(port);
        this.VOLTAGE_OFFSET = voltageOffset;
    }
    public OffsetAbsoluteAnalogEncoder(int port) {
        this(port, 0.0);
    }
    public double getUnadjustedVoltage() {
        return analogInput.getVoltage();
    }
    public double getAdjustedVoltage() {
        return getUnadjustedVoltage() - VOLTAGE_OFFSET;
    }
    public Rotation2d getAdjustedRotation2d() {
        return Rotation2d.fromDegrees(((getAdjustedVoltage() - Constants.SwerveConstants.MIN_VOLT) / (Constants.SwerveConstants.MAX_VOLT - Constants.SwerveConstants.MIN_VOLT)) * Constants.CIRCLE);
    }
}
