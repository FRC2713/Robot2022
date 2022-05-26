package frc.robot.subsystems.DriveIO;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface DriveIO {
  public static class DriveInputs implements LoggableInputs {
    public double gyroYawPositionRadians = 0.0;
    public double gyroPitchPositionRadians = 0.0;
    public double gyroRollPositionRadians = 0.0;
    public double leftVolts = 0.0;
    public double rightVolts = 0.0;

    @Override
    public void toLog(LogTable table) {
      table.put("Drive/YawRadians", gyroYawPositionRadians);
      table.put("Drive/PitchRadians", gyroPitchPositionRadians);
      table.put("Drive/RollRadians", gyroRollPositionRadians);
      table.put("Drive/LeftVoltage", leftVolts);
      table.put("Drive/RightVoltage", rightVolts);
    }

    @Override
    public void fromLog(LogTable table) {
      gyroYawPositionRadians = table.getDouble("Drive/YawRadians", gyroYawPositionRadians);
      gyroPitchPositionRadians = table.getDouble("Drive/PitchRadians", gyroPitchPositionRadians);
      gyroRollPositionRadians = table.getDouble("Drive/RollRadians", gyroRollPositionRadians);
      leftVolts = table.getDouble("Drive/LeftVoltage", leftVolts);
      rightVolts = table.getDouble("Drive/RightVoltage", rightVolts);
    }
  }

  public void updateInputs(DriveInputs inputs);

  public void setVoltage(DriveInputs inputs);

  public void getHeadingDegrees(DriveInputs inputs);

  // definitely a lot more things tbh
}
