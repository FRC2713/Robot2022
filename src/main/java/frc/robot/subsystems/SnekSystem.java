package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TunableNumber;

public class SnekSystem extends SubsystemBase {
  private CANSparkMax lowerSnek =
      new CANSparkMax(Constants.RobotMap.lowerSnek, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax upperSnek =
      new CANSparkMax(Constants.RobotMap.upperSnek, CANSparkMaxLowLevel.MotorType.kBrushless);
  private DigitalInput lowerLimit = new DigitalInput(Constants.RobotMap.lowerSnek);
  private DigitalInput upperLimit = new DigitalInput(Constants.RobotMap.upperSnek);

  private TunableNumber medianFilterSize = new TunableNumber("Snek/Median Filter Size", 5);
  private TunableNumber movingAverageFilterSize =
      new TunableNumber("Snek/Moving Avg Filter Size", 5);
  private TunableNumber singlePoleFilterTimeConstant =
      new TunableNumber("Snek/Single Pole Filter Time Constant", 0.1);

  private MedianFilter medianFilter = new MedianFilter((int) medianFilterSize.get());
  private LinearFilter movingAvgFilter =
      LinearFilter.movingAverage((int) movingAverageFilterSize.get());
  private LinearFilter singlePoleFilter =
      LinearFilter.singlePoleIIR(singlePoleFilterTimeConstant.get(), 0.02);

  private TunableNumber lowerSpeed = new TunableNumber("Snek/Lower Speed", 0);
  private TunableNumber upperSpeed = new TunableNumber("Snek/Upper Speed", 0);

  public SnekSystem() {
    // ya mum
    lowerSnek.restoreFactoryDefaults();
    upperSnek.restoreFactoryDefaults();

    lowerSnek.setSmartCurrentLimit(Constants.SnekConstants.currentLimit);
    upperSnek.setSmartCurrentLimit(Constants.SnekConstants.currentLimit);
  }

  public void loadSnek() {
    if (lowerLimit.get() && upperLimit.get()) {
      lowerSnek.set(0);
      upperSnek.set(0);
    } else if (!lowerLimit.get() && upperLimit.get()) {
      lowerSnek.set(0.5);
      upperSnek.set(0);
    } else {
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

  public void periodic() {
    if (Constants.tuningMode) {
      if (medianFilterSize.hasChanged()) {
        medianFilter = new MedianFilter((int) medianFilterSize.get());
      }
      if (movingAverageFilterSize.hasChanged()) {
        movingAvgFilter = LinearFilter.movingAverage((int) movingAverageFilterSize.get());
      }
      if (singlePoleFilterTimeConstant.hasChanged()) {
        singlePoleFilter = LinearFilter.singlePoleIIR(singlePoleFilterTimeConstant.get(), 0.02);
      }
    }

    upperSnek.set(upperSpeed.get());
    lowerSnek.set(lowerSpeed.get());

    double currentDraw = lowerSnek.getOutputCurrent();

    SmartDashboard.putNumber("Snek/Median Filter Value", medianFilter.calculate(currentDraw));
    SmartDashboard.putNumber(
        "Snek/Moving Avg Filter Value", movingAvgFilter.calculate(currentDraw));
    SmartDashboard.putNumber(
        "Snek/Single Pole Filter Value", singlePoleFilter.calculate(currentDraw));
  }
}
