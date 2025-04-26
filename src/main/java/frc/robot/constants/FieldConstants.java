package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import java.util.*;

import com.pathplanner.lib.util.FlippingUtil;


/**
 * <p>
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 * </p>
 * 
 * Made by team 6328
 */
public class FieldConstants {

  public static final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  public static final double fieldLength = layout.getFieldLength();
  public static final double fieldWidth = layout.getFieldWidth();
  public static final double startingLineX =
      Units.inchesToMeters(299.438); // Measured from the inside of starting line
  public static final double algaeDiameter = Units.inchesToMeters(16);
  public static final double coralDiameter = Units.inchesToMeters(4.5);

  public static class Processor {
    public static final Pose2d centerFace =
        new Pose2d(
           layout.getTagPose(16).get().getX(),
            0,
            Rotation2d.fromDegrees(90));
    public static final Pose2d opposingCenterFace =
        new Pose2d(
            layout.getTagPose(3).get().getX(),
            fieldWidth,
            Rotation2d.fromDegrees(-90));
  }

  public static class Barge {
    public static final double netWidth = Units.inchesToMeters(40.0);
    public static final double netHeight = Units.inchesToMeters(88.0);

    public static final Translation2d farCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
    public static final Translation2d middleCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
    public static final Translation2d closeCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

    // Measured from floor to bottom of cage
    public static final double deepHeight = Units.inchesToMeters(3.125);
    public static final double shallowHeight = Units.inchesToMeters(30.125);
  }

  public static class CoralStation {
    public static final double stationLength = Units.inchesToMeters(79.750);
    public static final Pose2d rightCenterFace =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(25.824),
            Rotation2d.fromDegrees(144.011 - 90));
    public static final Pose2d leftCenterFace =
        new Pose2d(
            rightCenterFace.getX(),
            fieldWidth - rightCenterFace.getY(),
            Rotation2d.fromRadians(-rightCenterFace.getRotation().getRadians()));
  }

  public static class Reef {
    public static final double faceLength = Units.inchesToMeters(36.792600);
    public static final Translation2d center =
        new Translation2d(Units.inchesToMeters(176.746), fieldWidth / 2.0);
    public static final double faceToZoneLine =
        Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

    public static final Map<Integer, Pose2d> centerFaces = Map.of(
      17, layout.getTagPose(17).get().toPose2d(),
      18, layout.getTagPose(18).get().toPose2d(),
      19, layout.getTagPose(19).get().toPose2d(),
      20, layout.getTagPose(20).get().toPose2d(),
      21, layout.getTagPose(21).get().toPose2d(),
      22, layout.getTagPose(22).get().toPose2d()
    );
    public static final List<Map<ReefLevel, Pose3d>> branchPositions =
        new ArrayList<>(); // Starting at the right branch facing the driver station in clockwise
    public static final Map<Integer, Pose2d> leftScorePositions = new HashMap<>();
    public static final Map<Integer, Pose2d> rightScorePositions = new HashMap<>();

    public static final Map<Integer, Integer> redIDToBlueID =
      Map.of(
        7, 18,
        9, 20,
        8, 19,
        10, 21,
        6, 17,
        11, 22
      );
    static {
      // Initialize branch positions
      for (int face = 0; face < 6; face++) {
          Pose2d poseDirection = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
          double adjustX = Units.inchesToMeters(30.738); //TODO: Fix these
          double adjustY = Units.inchesToMeters(6.469);

          var rightScorePose =
              new Pose2d(
                  new Translation2d(
                      poseDirection
                          .transformBy(new Transform2d(adjustX, adjustY, Rotation2d.kZero))
                          .getX(),
                      poseDirection
                          .transformBy(new Transform2d(adjustX, adjustY, Rotation2d.kZero))
                          .getY()),
                  new Rotation2d(
                      poseDirection.getRotation().getRadians()
                  )
              );
          var leftScorePose =
              new Pose2d(
                new Translation2d(
                  poseDirection
                      .transformBy(new Transform2d(adjustX, -adjustY, Rotation2d.kZero))
                      .getX(),
                  poseDirection
                      .transformBy(new Transform2d(adjustX, -adjustY, Rotation2d.kZero))
                      .getY()),
                  new Rotation2d(
                    poseDirection.getRotation().getRadians()
                  )
                );

          leftScorePositions.put(faceToAprilTagID(face), leftScorePose);
          rightScorePositions.put(faceToAprilTagID(face), rightScorePose);
      }
    }

    public static Pose2d getScorePose(int tagID, boolean isLeft, boolean shouldFlipPose) {
      if (shouldFlipPose) {
        tagID = redIDToBlueID.get(tagID);
        return FlippingUtil.flipFieldPose(isLeft ? leftScorePositions.get(tagID) : rightScorePositions.get(tagID));
      }
      
      return isLeft ? leftScorePositions.get(tagID) : rightScorePositions.get(tagID);
    }

    private static int faceToAprilTagID(int face) {
      return switch (face) {
        case 0 -> 18;
        case 1 -> 19;
        case 2 -> 20;
        case 3 -> 21;
        case 4 -> 22;
        case 5 -> 17;
        default -> -1;
      };
    }
  }

  public static class StagingPositions {
    // Measured from the center of the ice cream
    public static final double separation = Units.inchesToMeters(72.0);
    public static final Pose2d middleIceCream =
        new Pose2d(Units.inchesToMeters(48), fieldWidth / 2.0, Rotation2d.kZero);
    public static final Pose2d leftIceCream =
        new Pose2d(Units.inchesToMeters(48), middleIceCream.getY() + separation, Rotation2d.kZero);
    public static final Pose2d rightIceCream =
        new Pose2d(Units.inchesToMeters(48), middleIceCream.getY() - separation, Rotation2d.kZero);
  }

  public enum ReefLevel {
    L1(0, Units.inchesToMeters(25.0), 0),
    L2(1, Units.inchesToMeters(31.875 - Math.cos(Math.toRadians(35.0)) * 0.625), -35),
    L3(2, Units.inchesToMeters(47.625 - Math.cos(Math.toRadians(35.0)) * 0.625), -35),
    L4(3, Units.inchesToMeters(72), -90);

    ReefLevel(int levelNumber, double height, double pitch) {
      this.levelNumber = levelNumber;
      this.height = height;
      this.pitch = pitch; // Degrees
    }

    public static ReefLevel fromLevel(int level) {
      return Arrays.stream(values())
          .filter(height -> height.ordinal() == level)
          .findFirst()
          .orElse(L4);
    }

    public final int levelNumber;
    public final double height;
    public final double pitch;
  }


  public record CoralObjective(int branchId, ReefLevel reefLevel) {}

  public record AlgaeObjective(int id, boolean low) {
    public AlgaeObjective(int id) {
      this(id, id % 2 == 1);
    }
  }
}
