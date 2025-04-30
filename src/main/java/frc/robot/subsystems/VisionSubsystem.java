package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.constants.RobotConstants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {

    public PhotonCamera camera;
    private PhotonPoseEstimator photonPoseEstimator;
    public static PhotonPipelineResult result;
    private List<PhotonPipelineResult> allResults;

    private CommandSwerveDrivetrain drivetrain = RobotContainer.getDrivetrain();

    public PIDController rotController = new PIDController(VisionConstants.VR_kP, VisionConstants.VR_kI,
            VisionConstants.VR_kD);
    public PIDController driveControllerY = new PIDController(VisionConstants.VY_kP,
            VisionConstants.VY_kI, VisionConstants.VY_kD);
    public PIDController driveControllerX = new PIDController(VisionConstants.VX_kP,
            VisionConstants.VX_kI, VisionConstants.VX_kD);

    private AprilTagFieldLayout layout = VisionConstants.layout;

    private boolean shouldWarn = true;

    public VisionSubsystem(String cameraName, Transform3d cameraOffset) {
        camera = new PhotonCamera(cameraName);

        photonPoseEstimator = new PhotonPoseEstimator(
                layout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, //TODO: Update this setting in PhotonVision
                cameraOffset
            );
    }


    /**
     * Used to get the X position of the best target.
     * @return The X positon of the best target.
     *         If no target is present, return an empty optional.
     */
    public Optional<Double> getX() {
        if (hasTargets()) {
            return Optional.of(result.getBestTarget().getBestCameraToTarget().getTranslation().getX());
        }
        return Optional.empty();
    } 

    /**
     * Used to get the Y position of the best target.
     * @return The Y position of the best target.
     *         If no target is present, return an empty optional.
     */
    public Optional<Double> getY() {
        if (hasTargets()) {
            return Optional.of(result.getBestTarget().bestCameraToTarget.getY());
        } else
            return Optional.empty();
    }

    /**
     * Gets the yaw of the best target.
     * @return The yaw of the best target.
     *         If no target is present, return an empty optional.
     */
    public Optional<Double> getYaw() {
        if (hasTargets()) {
            return Optional.of(result.getBestTarget().getYaw());
        }
        return Optional.empty();
    }

    /**
     * Gets the pitch of the best target.
     * @return The pitch of the best target.
     *         If no target is present, return an empty optional.
     */
    public Optional<Double> getPitch() {
        if (hasTargets()) {
            return Optional.of(result.getBestTarget().getPitch());
        }
        return Optional.empty();
    }
    
    /**
     * Used to aim the robot at the apriltag.
     * @param altRotation
     * @return the PID output for the rotation.
     */
    public double visionTargetPIDCalc(double altRotation) {
        boolean target = hasTargets();

        if (target && getYaw().isPresent()) {
            return -rotController.calculate(layout.getTagPose(result.getBestTarget().getFiducialId())
                    .get().getRotation().getZ());
        }
 
        return altRotation;
    }

    public double visionPara(double altRotation, boolean visionMode, double gyro) {
        if (visionMode) {
            if (hasTargets()) {
                return rotController.calculate(gyro % 360
                        - Math.toDegrees(layout.getTagPose(result.getBestTarget().getFiducialId())
                                .get().getRotation().getMeasureZ().magnitude()));
            }
        }
        if (!visionMode) {
            return altRotation;
        }
        return altRotation;
    }


    /**
     * Used to make the robot drive to a set distance away from an apriltag.
     * 
     * @param altDrive  The input from the controller for then no tag is present.
     * @param distance  How far away from the apriltag do we want to be.
     * 
     * @return
     */
    public double visionYDrive(double altDrive, double distance) {
        if (getX().isPresent() && hasTargets()) {
            return driveControllerX.calculate(getYaw().get() - distance);
        } else {
            return altDrive;
        }
    }

    public double visionXDrive(double altDrive, double distance) {
        if (getX().isPresent() && hasTargets()) {
            return driveControllerY.calculate(getX().get() - distance);
        } else {
            return altDrive;
        }
    }


    public boolean hasTargets() {
        return result.hasTargets();
    }

    public boolean hasTarget(int id) {
        var target = getTargetByID(id);
        if (target.isEmpty()) return false;

        return true;
    }

    public PhotonPipelineResult getLatestResult() {
        allResults = camera.getAllUnreadResults();
        int size = allResults.size();
        return size > 0 ? allResults.get(size - 1) : new PhotonPipelineResult();
    }

    public Optional<Integer> getTargetID() {
        if (!hasTargets()) return Optional.empty();

        return Optional.of(result.getBestTarget().getFiducialId());
    }

    private Optional<PhotonTrackedTarget> getTargetByID(int id) {
        if (!hasTargets()) return Optional.empty();

        for (PhotonTrackedTarget target : result.getTargets()) {
            if (target.getFiducialId() == id) {
                return Optional.of(target);
            }
        }
        return Optional.empty();

    }

    /**
     * Clears targets that are too far away or have a high pose ambiguity.
     * 
     * @author Team 1108
     */
    public PhotonPipelineResult clearEvilTargets(PhotonPipelineResult poseResult) {
        if (poseResult.hasTargets()) {
            for (PhotonTrackedTarget target : poseResult.getTargets()) {
                var tagPose = layout.getTagPose(target.getFiducialId());

                if (tagPose.isEmpty()) {
                    result.targets.remove(target);
                }

                var distance = PhotonUtils.getDistanceToPose(drivetrain.getState().Pose, tagPose.get().toPose2d());
                // Check if the target is too far away or has a high pose ambiguity
                if (target.getPoseAmbiguity() > .2 || distance > VisionConstants.MAX_DISTANCE_METERS) {
                    poseResult.targets.remove(target);
                }
            }
        }

        return poseResult;
    }

    @Override
    public void periodic() {
        if (!camera.isConnected()) {
            if (shouldWarn) {
                DriverStation.reportWarning("Camera not connected", false);
                shouldWarn = false;
            }

            return;
        }

        result = getLatestResult();


        PhotonPipelineResult poseResult = result;
        if (!poseResult.hasTargets()) return;

        poseResult = clearEvilTargets(poseResult);

        if (!poseResult.hasTargets()) return;

        Optional<EstimatedRobotPose> estimatedPose = photonPoseEstimator.update(poseResult);
        if (estimatedPose.isEmpty()) return;

        drivetrain.addVisionMeasurement(estimatedPose.get().estimatedPose.toPose2d(), estimatedPose.get().timestampSeconds);
    }

}