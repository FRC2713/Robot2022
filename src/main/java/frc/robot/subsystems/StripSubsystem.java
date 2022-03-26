package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;

/** Add your docs here. */
public class StripSubsystem extends SubsystemBase {

  // private static StripSubsystem strip;

  private Spark stripSpark;

  public StripSubsystem() {
    stripSpark = new Spark(RobotMap.stripPort);
  }

  public void setColor(Pattern pattern) {
    stripSpark.set(pattern.getValue());
  }

  public void setAllianceColor(StripSubsystem strip) {
    DriverStation.Alliance alliance = DriverStation.getAlliance();
    if (alliance == DriverStation.Alliance.Blue) {
      strip.setColor(Pattern.Blue);
    } else if (alliance == DriverStation.Alliance.Red) {
      strip.setColor(Pattern.Red);
    } else {
      strip.setColor(Pattern.RainbowRainbow);
    }
  }

  public void stop() {
    stripSpark.set(0);
  }

  public enum Pattern {
    RainbowRainbow(-0.99),
    RainbowParty(-0.97),
    RainbowOcean(-0.95),
    RainbowLava(-0.93),
    RainbowForest(-0.91),
    RainbowGlitter(-0.89),
    Confetti(-0.87),
    ShotRed(-0.85),
    ShotBlue(-0.83),
    ShotWhite(-0.81),
    SinelonRainbow(-0.79),
    SinelonParty(-0.77),
    SinelonOcean(-0.75),
    SinelonLava(-0.73),
    SinelonForest(-0.71),
    BPMRainbow(-0.69),
    BPMParty(-0.67),
    BPMOcean(-0.65),
    BPMLava(-0.63),
    BPMForest(-0.61),
    FireMedium(-0.59),
    FireLarge(-0.57),
    TwinklesRainbow(-0.55),
    TwinklesParty(-0.53),
    TwinklesOcean(-0.51),
    TwinklesLava(-0.49),
    TwinklesForest(-0.47),
    ColorWavesRainbow(-0.45),
    ColorWavesParty(-0.43),
    ColorWavesOcean(-0.41),
    ColorWavesLava(-0.39),
    ColorWavesForest(-0.37),
    LarsonRed(-0.35),
    LarsonGray(-0.33),
    LightChaseRed(-0.31),
    LightChaseBlue(-0.29),
    LightChaseGray(-0.27),
    HeartbeatRed(-0.25),
    HeartbeatBlue(-0.23),
    HeartbeatWhite(-0.21),
    HeartbeatGray(-0.19),
    BreathRed(-0.17),
    BreathBlue(-0.15),
    BreathGray(-0.13),
    StrobeRed(-0.11),
    StrobeBlue(-0.09),
    StrobeGold(-0.07),
    StrobeWhite(-0.05),
    Color1BlendToBlack(-0.03),
    Color1Larson(-0.01),
    Color1LightChase(0.01),
    Color1HeartbeatSlow(0.03),
    Color1HeartbeatMedium(0.05),
    Color1HeartbeatFast(0.07),
    Color1BreathSlow(0.09),
    Color1BreathFast(0.11),
    Color1Shot(0.13),
    Color1Strobe(0.15),
    Color2BlendToBlack(0.17),
    Color2Larson(0.19),
    Color2LightChase(0.21),
    Color2HeartbeatSlow(0.23),
    Color2HeartbeatMedium(0.25),
    Color2HeartbeatFast(0.27),
    Color2BreathSlow(0.29),
    Color2BreathFast(0.31),
    Color2Shot(0.33),
    Color2Strobe(0.35),
    Color1And2Spark1On2(0.37),
    Color1And2Spark2On1(0.39),
    Color1And2Gradient(0.41),
    Color1And2BPM(0.43),
    Color1And2EndtoEnd1To2(0.45),
    Color1And2EndtoEnd(0.47),
    Color1And2NoBlend(0.49),
    Color1And2Twinkles(0.51),
    Color1And2ColorWaves(0.53),
    Color1And2Sinelon(0.55),
    HotPink(0.57),
    DarkRed(0.59),
    Red(0.61),
    RedOrange(0.63),
    Orange(0.65),
    Gold(0.67),
    Yellow(0.69),
    LawnGreen(0.71),
    Lime(0.73),
    DarkGreen(0.75),
    Green(0.77),
    BlueGreen(0.79),
    Aqua(0.81),
    SkyBlue(0.83),
    DarkBlue(0.85),
    Blue(0.87),
    BlueViolet(0.89),
    Violet(0.91),
    White(0.93),
    Gray(0.95),
    DarkGray(0.97),
    Black(0.99);

    private double value;

    Pattern(double value) {
      this.value = value;
    }

    public double getValue() {
      return this.value;
    }
  }
}
