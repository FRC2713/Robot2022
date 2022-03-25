package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  private double limelightHeightInches;
  private NetworkTable getTable() {
    return NetworkTableInstance.getDefault().getTable("limelight");
  }

  private NetworkTableEntry getEntry(String entryName) {
    return getTable().getEntry(entryName);
  }

  private double getValue(String entryName) {
    return getEntry(entryName).getDouble(0);
  }

  private void setValue(String entryName, double value) {
    getEntry(entryName).setNumber(value);
  }

  /** @return true if the Limelight has any valid targets */
  public boolean hasValidTargets() {
    return getValue("tv") == 1;
  }

  public double getTargetValue() {
    return getValue("tv");
  }

  /** @return horizontal offset from crosshair to target (-27 degrees to 27 degrees) */
  public double getHorizontalOffset() {
    return getValue("tx");
  }

  /** @return vertical offset from crosshair to target (-20.5 degrees to 20.5 degrees) */
  public double getVerticalOffset() {
    return getValue("ty");
  }

  /** @return target area (0% of image to 100% of image) */
  public double getTargetArea() {
    return getValue("ta");
  }

  /** @return skew or rotation (-90 degrees to 0 degrees) */
  public double getSkewOrRotation() {
    return getValue("ts");
  }

  /** @return pipeline's latency contribution (ms); add 11ms for image capture */
  public double getLatency() {
    return getValue("tl");
  }

  /** @return sidelength of shortest side of the fitted bounding box (pixels) */
  public double getShortSideLength() {
    return getValue("tshort");
  }

  /** @return sidelength of longest side of the fitted bounding box (pixels) */
  public double getLongSideLength() {
    return getValue("tlong");
  }

  /** @return Horizontal sidelength of the rough bounding box (0 - 320 pixels) */
  public double getHorizontalSidelength() {
    return getValue("thor");
  }

  /** @return Vertical sidelength of the rough bounding box (0 - 320 pixels) */
  public double getVerticalSidelength() {
    return getValue("tvert");
  }

  /** @return True active pipeline index of the camera (0 .. 9) */
  public double getPipeline() {
    return getValue("getpipe");
  }

//   /**
//    * @return Results of a 3D position solution, 6 numbers: Translation (x,y,y)
//    *     Rotation(pitch,yaw,roll)
//    */
//   public Double[] getCamtran() {
//     return getEntry("camtran").getDoubleArray(new Double[] {});
//   }

  public enum LedMode {
    PIPELINE(0),
    FORCE_OFF(1),
    FORCE_BLINK(2),
    FORCE_ON(3),
    UNKNOWN(-1);
    public double value;

    LedMode(double value) {
      this.value = value;
    }
  }

  /** @return The current LED mode set on the Limelight */
  public LedMode getLedMode() {
    double mode = getValue("ledMode");
    if (mode == 0) {
      return LedMode.PIPELINE; // Uses the LED mode set in the pipeliine
    } else if (mode == 1) {
      return LedMode.FORCE_OFF;
    } else if (mode == 2) {
      return LedMode.FORCE_BLINK;
    } else if (mode == 3) {
      return LedMode.FORCE_ON;
    } else {
      System.out.println("[Limelight] UNKNOWN LED MODE -- " + mode);
      return LedMode.UNKNOWN;
    }
  }

  /** @param mode The LED Mode to set on the Limelight */
  public void setLedMode(LedMode mode) {
    if (mode != LedMode.UNKNOWN) {
      setValue("ledMode", mode.value);
    }
  }

  public double getDistanceFromGoal(){
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry ty = table.getEntry("ty");
  double targetOffsetAngle_Vertical = ty.getDouble(0.0);

  // how many degrees back is your limelight rotated from perfectly vertical?
  //CHECK
  double limelightMountAngleDegrees = 25.0;

  // distance from the center of the Limelight lens to the floor
  //CHANGE how high up it is
  double limelightLensHeightInches = 20.0;

  // distance from the target to the floor
  double goalHeightInches = 104.0;

  double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
  double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

  //calculate distance
  return (goalHeightInches - limelightHeightInches)/Math.tan(angleToGoalRadians);
  }
//   public enum CamMode {
//     VISION_CAM(0),
//     DRIVER_CAM(1),
//     UNKNOWN(-1);
//     public double value;

//     CamMode(double value) {
//       this.value = value;
//     }
//   }

//   /** @return The current LED mode set on the Limelight */
//   public CamMode getCamMode() {
//     double mode = getValue("camMode");
//     if (mode == 0) {
//       return CamMode.VISION_CAM;
//     } else if (mode == 1) {
//       return CamMode.DRIVER_CAM;
//     } else {
//       System.out.println("[Limelight] UNKNOWN CAMERA MODE -- " + mode);
//       return CamMode.UNKNOWN;
//     }
//   }

//   /** @param mode The LED Mode to set on the Limelight */
//   public void setCamMode(CamMode mode) {
//     if (mode != CamMode.UNKNOWN) {
//       setValue("camMode", mode.value);
//     }
//   }

  public enum Pipeline {
    PIPELINE0(0),
    PIPELINE1(1),
    // PIPELINE2(2),
    // PIPELINE3(3),
    // PIPELINE4(4),
    // PIPELINE5(5),
    // PIPELINE6(6),
    // PIPELINE7(7),
    // PIPELINE8(8),
    // PIPELINE9(9),
    UNKNOWN(-1);

    public double value;

    Pipeline(double value) {
      this.value = value;
    }
  }

  public Pipeline getCurrentPipeline() {
    double mode = getValue("pipeline");
    if (mode == 0) {
      return Pipeline.PIPELINE0;
    } else if (mode == 1) {
      return Pipeline.PIPELINE1;
    // } else if (mode == 2) {
    //   return Pipeline.PIPELINE2;
    // } else if (mode == 3) {
    //   return Pipeline.PIPELINE3;
    // } else if (mode == 4) {
    //   return Pipeline.PIPELINE4;
    // } else if (mode == 5) {
    //   return Pipeline.PIPELINE5;
    // } else if (mode == 6) {
    //   return Pipeline.PIPELINE6;
    // } else if (mode == 7) {
    //   return Pipeline.PIPELINE7;
    // } else if (mode == 8) {
    //   return Pipeline.PIPELINE8;
    // } else if (mode == 9) {
    //   return Pipeline.PIPELINE9;
    } else {
      System.out.println("[Limelight] UNKNOWN Pipeline -- " + mode);
      return Pipeline.UNKNOWN;
    }
  }

  /** @param mode The LED Mode to set on the Limelight */
  public void setPipeline(Pipeline mode) {
    if (mode != Pipeline.UNKNOWN) {
      setValue("pipeline", mode.value);
    }
  }

//   public enum StreamMode {
//     STANDARD(0),
//     PIP_MAIN(1),
//     PIP_SECONDARY(2),
//     UNKNOWN(-1);

//     public double value;

//     StreamMode(double value) {
//       this.value = value;
//     }
//   }

//   /** @return The current LED mode set on the Limelight */
//   public StreamMode getCurrentStreamMode() {
//     double mode = getValue("stream");
//     if (mode == 0) {
//       return StreamMode.STANDARD; // Side-by-side streams if a webcam is attached to Limelight
//     } else if (mode == 1) {
//       return StreamMode
//           .PIP_MAIN; // The secondary camera stream is placed in the lower-right corner of the
//       // primary camera stream
//     } else if (mode == 2) {
//       return StreamMode.PIP_SECONDARY;
//     } else {
//       System.out.println("[Limelight] UNKNOWN StreamMode -- " + mode);
//       return StreamMode.UNKNOWN;
//     }
//   }

//   /** @param mode The LED Mode to set on the Limelight */
//   public void setStreamMode(StreamMode mode) {
//     if (mode != StreamMode.UNKNOWN) {
//       setValue("stream", mode.value);
//     }
//   }

  public enum SnapshotMode {
    OFF(0),
    TWO_PER_SECOND(1),
    UNKNOWN(-1);

    public double value;

    SnapshotMode(double value) {
      this.value = value;
    }
  }

  /** @return The current LED mode set on the Limelight */
  public SnapshotMode getCurrentSnapShotMode() {
    double mode = getValue("snapshot");
    if (mode == 0) {
      return SnapshotMode.OFF;
    } else if (mode == 1) {
      return SnapshotMode.TWO_PER_SECOND;
    } else {
      System.out.println("[Limelight] UNKNOWN SnapshotMode -- " + mode);
      return SnapshotMode.UNKNOWN;
    }
  }

  /** @param mode The LED Mode to set on the Limelight */
  public void setSnapshotMode(SnapshotMode mode) {
    if (mode != SnapshotMode.UNKNOWN) {
      setValue("snapshot", mode.value);
    }
  }

  public void periodic() {
  }
}