package frc.robot.util;


import java.util.HashMap;
import java.util.Map;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.constants.RobotConstants.VisionConstants;

public class Poses {
    private static final AprilTagFieldLayout layout = VisionConstants.layout;

    private static final Translation2d center = new Translation2d(
        Units.inchesToMeters(176.746), layout.getFieldWidth() / 2
    );

    private static final Map<Integer, Integer> faceToTagId = Map.of(
        0, 18,
        1, 19,
        2, 20,
        3, 21,
        4, 22,
        5, 17
    );

    private static final Map<Integer, Pose2d> leftScorePoses = new HashMap<>();
    private static final Map<Integer, Pose2d> rightScorePoses = new HashMap<>();

    static {
        //General structue for this is by 6328, modifed for our needs
        for (int face = 0; face < 6; face++) {
            Pose2d poseDirection = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
            double adjustX = Units.inchesToMeters(30.738); //TODO: update these
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
                        poseDirection.getRotation()
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
                        poseDirection.getRotation()
                );

            leftScorePoses.put(faceToTagId.get(face), leftScorePose);
            rightScorePoses.put(faceToTagId.get(face), rightScorePose);
        }
    }

    public static Pose2d getScorePose(int tagId, boolean isLeft, boolean shouldFlip) {
        Pose2d target = isLeft ? leftScorePoses.get(tagId) : rightScorePoses.get(tagId);

        if (shouldFlip) {
            return FlippingUtil.flipFieldPose(target);
        } else {
            return target;
        }
    }
}
