package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.Util.Geometry.Direction;

public class FieldConstants {
  // Field dimensions
  public static final double fieldLength = Units.inchesToMeters(54.0 * 12.0);
  public static final double fieldWidth = Units.inchesToMeters(27.0 * 12.0);
  public static final double hangarLength = Units.inchesToMeters(128.75);
  public static final double hangarWidth = Units.inchesToMeters(116.0);

  // Vision target
  public static final double visionTargetDiameter = Units.inchesToMeters(4.0 * 12.0 + 5.375);

  /* Bottom of tape */
  public static final double visionTargetHeightLower = Units.inchesToMeters(8.0 * 12 + 5.625);

  /* Top of tape */
  public static final double visionTargetHeightUpper =
      visionTargetHeightLower + Units.inchesToMeters(2.0);

  // Dimensions of hub and tarmac
  public static final Rotation2d centerLineAngle = Rotation2d.fromDegrees(66.0);
  public static final Translation2d hubCenter =
      new Translation2d(fieldLength / 2.0, fieldWidth / 2.0);
  public static final double tarmacInnerDiameter = Units.inchesToMeters(219.25);
  public static final double tarmacOuterDiameter = Units.inchesToMeters(237.31);
  public static final double tarmacFenderToTip = Units.inchesToMeters(84.75);

  /* If the tarmac formed a full octagon */
  public static final double tarmacFullSideLength = tarmacInnerDiameter * (Math.sqrt(2.0) - 1.0);

  /* Length of tape marking outside of tarmac */
  public static final double tarmacMarkedSideLength = Units.inchesToMeters(82.83);

  /* Length removed because of corner cutoff */
  public static final double tarmacMissingSideLength =
      tarmacFullSideLength - tarmacMarkedSideLength;

  public static final double hubSquareLength = tarmacOuterDiameter - (tarmacFenderToTip * 2.0);

  // Reference rotations (angle from hub to each reference point and fender side)
  public static final Rotation2d referenceARotation =
      Rotation2d.fromDegrees(180.0)
          .minus(centerLineAngle)
          .plus(Rotation2d.fromDegrees(360.0 / 16.0));
  public static final Rotation2d referenceBRotation =
      referenceARotation.rotateBy(Rotation2d.fromDegrees(360.0 / 8.0));
  public static final Rotation2d referenceCRotation =
      referenceBRotation.rotateBy(Rotation2d.fromDegrees(360.0 / 8.0));
  public static final Rotation2d referenceDRotation =
      referenceCRotation.rotateBy(Rotation2d.fromDegrees(360.0 / 8.0));
  public static final Rotation2d fenderARotation =
      referenceARotation.rotateBy(Rotation2d.fromDegrees(360.0 / 16.0));
  public static final Rotation2d fenderBRotation =
      fenderARotation.rotateBy(Rotation2d.fromDegrees(90.0));

  // Reference points (centered of the sides of the tarmac if they formed a
  // complete octagon, plus
  // edges of fender)
  public static final Pose2d referenceA =
      new Pose2d(hubCenter, referenceARotation)
          .transformBy(Util.Geometry.transformFromTranslation(tarmacInnerDiameter / 2.0, 0.0));
  public static final Pose2d referenceB =
      new Pose2d(hubCenter, referenceBRotation)
          .transformBy(Util.Geometry.transformFromTranslation(tarmacInnerDiameter / 2.0, 0.0));
  public static final Pose2d referenceC =
      new Pose2d(hubCenter, referenceCRotation)
          .transformBy(Util.Geometry.transformFromTranslation(tarmacInnerDiameter / 2.0, 0.0));
  public static final Pose2d referenceD =
      new Pose2d(hubCenter, referenceDRotation)
          .transformBy(Util.Geometry.transformFromTranslation(tarmacInnerDiameter / 2.0, 0.0));
  private static final Pose2d fenderA =
      new Pose2d(hubCenter, fenderARotation)
          .transformBy(
              Util.Geometry.transformFromTranslation(
                  hubSquareLength / 2.0 + Units.inchesToMeters(0.0), 0.0));
  private static final Pose2d fenderB =
      new Pose2d(hubCenter, fenderBRotation)
          .transformBy(
              Util.Geometry.transformFromTranslation(
                  hubSquareLength / 2.0 + Units.inchesToMeters(0.0), 0.0));

  // Cargo points
  public static final double cornerToCargoY = Units.inchesToMeters(15.56);
  public static final double referenceToCargoY = (tarmacFullSideLength / 2.0) - cornerToCargoY;
  public static final double referenceToCargoX = Units.inchesToMeters(40.44);
  public static final Pose2d cargoA =
      referenceA.transformBy(
          Util.Geometry.transformFromTranslation(referenceToCargoX, -referenceToCargoY));
  public static final Pose2d cargoB =
      referenceA.transformBy(
          Util.Geometry.transformFromTranslation(referenceToCargoX, referenceToCargoY));
  public static final Pose2d cargoC =
      referenceB.transformBy(
          Util.Geometry.transformFromTranslation(referenceToCargoX, referenceToCargoY));
  public static final Pose2d cargoD =
      referenceC.transformBy(
          Util.Geometry.transformFromTranslation(referenceToCargoX, -referenceToCargoY));
  //   .transformBy(
  //       Util.Geometry.transformFromTranslation(0, Units.inchesToMeters(13))); // 0, 6, 13
  public static final Pose2d cargoE =
      Util.Geometry.offsetDrivetrainFromPose(
          referenceD.transformBy(
              Util.Geometry.transformFromTranslation(referenceToCargoX, -referenceToCargoY)),
          Direction.NEGATIVE);
  public static final Pose2d cargoF =
      referenceD.transformBy(
          Util.Geometry.transformFromTranslation(referenceToCargoX, referenceToCargoY));

  // Terminal cargo point
  public static final Rotation2d terminalOuterRotation = Rotation2d.fromDegrees(133.75);
  public static final double terminalLength = Units.inchesToMeters(324.0 - 256.42);
  public static final double terminalWidth =
      Math.tan(Rotation2d.fromDegrees(180.0).minus(terminalOuterRotation).getRadians())
          * terminalLength;
  public static final Pose2d terminalCenter =
      new Pose2d(
          new Translation2d(terminalLength / 2.0, terminalWidth / 2.0),
          terminalOuterRotation.minus(Rotation2d.fromDegrees(90.0)));
  public static final double terminalCargoOffset = Units.inchesToMeters(10.43);
  public static final Pose2d cargoG =
      Util.Geometry.offsetDrivetrainFromPose(
          terminalCenter.transformBy(
              new Transform2d(
                  new Translation2d(terminalCargoOffset, 0), Rotation2d.fromDegrees(180))),
          Direction.NEGATIVE,
          Units.inchesToMeters(20));

  // Starting points
  public static class StartingPoints {
    public static final Pose2d tarmacA =
        referenceA.transformBy(Util.Geometry.transformFromTranslation(-0.5, 0.7));
    public static final Pose2d tarmacB =
        referenceB.transformBy(Util.Geometry.transformFromTranslation(-0.5, -0.2));
    public static final Pose2d tarmacC =
        referenceC.transformBy(Util.Geometry.transformFromTranslation(-0.5, -0.1));
    public static final Pose2d tarmacD =
        // referenceD.transformBy(Util.Geometry.transformFromTranslation(-0.5, -0.7));
        Util.Geometry.offsetDrivetrainFromPose(
            new Pose2d(
                new Translation2d(
                    cargoE.getTranslation().getX(), referenceD.getTranslation().getY()),
                cargoE.getRotation()),
            Direction.NEGATIVE);

    public static final Pose2d fenderA =
        Util.Geometry.offsetDrivetrainFromPose(FieldConstants.fenderA, Direction.POSITIVE);
    public static final Pose2d fenderB =
        Util.Geometry.offsetDrivetrainFromPose(FieldConstants.fenderB, Direction.POSITIVE);
  }
}
