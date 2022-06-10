package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class MeasureKS extends CommandBase {
  private static final double stepVolts = 0.1;
  private static final double startDelaySec = 2.0;

  private DoubleSupplier velocitySupplier;
  private DoubleConsumer voltageSetter;

  private final Timer timer = new Timer();
  private double voltage = 0;

  public MeasureKS(Subsystem s, DoubleSupplier velocitySupplier, DoubleConsumer voltageSetter) {
    addRequirements(s);
    this.velocitySupplier = velocitySupplier;
    this.voltageSetter = voltageSetter;
  }

  public void initialize() {
    timer.reset();
    timer.start();
    voltageSetter.accept(0);
    voltage = 0;
  }

  public void execute() {
    if (timer.get() < startDelaySec) {
      voltageSetter.accept(0);
    } else {
      voltage = (timer.get() - startDelaySec) * stepVolts;
      voltageSetter.accept(voltage);
      SmartDashboard.putNumber("MKS voltage applied", voltage);
    }
  }

  public void end() {
    SmartDashboard.putNumber("MKS", voltage);
    voltageSetter.accept(0);
  }

  public boolean isFinished() {
    return velocitySupplier.getAsDouble() > 1e-2;
  }
}
