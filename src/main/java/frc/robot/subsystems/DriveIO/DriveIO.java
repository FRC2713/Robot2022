package frc.robot.subsystems.DriveIO;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface DriveIO {

  public static class DriveInputs implements LoggableInputs {
    public double gyroYawPositionRadians = 0.0;
    public double gyroPitchPositionRadians = 0.0;
    public double gyroRollPositionRadians = 0.0;

    public double leftVolts = 0.0;
    public double frontLeftCurrent = 0.0;
    public double leftEncPosition = 0.0;

    public double rightVolts = 0.0;
    public double frontRightCurrent = 0.0;
    public double rightEncPosition = 0.0;

    public double gyroHeadingDegrees = 0.0;

    @Override
    public void toLog(LogTable table) {
      table.put("Drive/YawRadians", gyroYawPositionRadians);
      table.put("Drive/PitchRadians", gyroPitchPositionRadians);
      table.put("Drive/RollRadians", gyroRollPositionRadians);

      table.put("Drive/LeftVoltage", leftVolts);
      table.put("Drive/FrontLeftCurrent", frontLeftCurrent);
      table.put("Drive/leftEncPosition", leftEncPosition);

      table.put("Drive/RightVoltage", rightVolts);
      table.put("Drive/FrontRightCurrent", frontRightCurrent);
      table.put("Drive/rightEncPosition", rightEncPosition);

      table.put("Drive/GyroHeadingDegrees", gyroHeadingDegrees);
    }

    @Override
    public void fromLog(LogTable table) {
      gyroYawPositionRadians = table.getDouble("Drive/YawRadians", gyroYawPositionRadians);
      gyroPitchPositionRadians = table.getDouble("Drive/PitchRadians", gyroPitchPositionRadians);
      gyroRollPositionRadians = table.getDouble("Drive/RollRadians", gyroRollPositionRadians);

      leftVolts = table.getDouble("Drive/LeftVoltage", leftVolts);
      leftEncPosition = table.getDouble("Drive/leftEncPosition", leftEncPosition);
      frontLeftCurrent = table.getDouble("Drive/FrontLeftCurrent", frontLeftCurrent);

      rightVolts = table.getDouble("Drive/RightVoltage", rightVolts);
      rightEncPosition = table.getDouble("Drive/rightEncPosition", rightEncPosition);
      frontRightCurrent = table.getDouble("Drive/FrontRightCurrent", frontRightCurrent);

      gyroHeadingDegrees = table.getDouble("Drive/GyroHeadingDegrees", gyroHeadingDegrees);
    }
  }

  public void updateInputs(DriveInputs inputs);

  public void setVoltage(double leftVolts, double rightVolts);

  // definitely a lot more things tbh but ill add them as I learn I need them
}
