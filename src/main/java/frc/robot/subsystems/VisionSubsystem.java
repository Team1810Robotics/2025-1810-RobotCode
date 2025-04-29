package frc.robot.subsystems;

import java.util.ArrayList;
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
import frc.robot.util.constants.FieldConstants;
import frc.robot.util.constants.RobotConstants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {

    public PhotonCamera camera;
    private PhotonPoseEstimator photonPoseEstimator;
    public static PhotonPipelineResult result;
    private List<PhotonPipelineResult> allResults;

    private CommandSwerveDrivetrain drivetrain;

    public PIDController rotController = new PIDController(VisionConstants.VR_kP, VisionConstants.VR_kI,
            VisionConstants.VR_kD);
    public PIDController driveControllerY = new PIDController(VisionConstants.VY_kP,
            VisionConstants.VY_kI, VisionConstants.VY_kD);
    public PIDController driveControllerX = new PIDController(VisionConstants.VX_kP,
            VisionConstants.VX_kI, VisionConstants.VX_kD);

    AprilTagFieldLayout layout = FieldConstants.layout;

    private boolean shouldWarn = true;

    public VisionSubsystem(String cameraName, Transform3d cameraOffset, CommandSwerveDrivetrain drivetrain) {
        camera = new PhotonCamera(cameraName);

        photonPoseEstimator = new PhotonPoseEstimator(
                layout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, //TODO: Update this setting in PhotonVision
                cameraOffset);
    }


    public Optional<Double> getRange() {
        if (hasTargets()) {
            // Get the range to the target using the best target's pose
            return Optional.of(result.getBestTarget().getBestCameraToTarget().getTranslation().getX());
        }
        return Optional.empty();
    } 


    public Optional<Double> getXRange() {
        if (hasTargets()) {
            return Optional.of(result.getBestTarget().bestCameraToTarget.getY());
        } else
            return Optional.empty();
    }

    public Optional<Double> getTagRYaw() {
        if (hasTargets()) {
            return Optional.of(layout.getTagPose(result.getBestTarget().getFiducialId()).get()
                    .getRotation().getZ());
        }
        return Optional.empty();
    }


    
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
     * @param driveMode True/False input for if we do want to drive towards the
     *                  apriltag.
     * @return
     */
    public double visionYDrive(double altDrive, double distance) {
        if (getRange().isPresent() && hasTargets()) {
            return driveControllerX.calculate(getYaw().get() - distance);
        } else {
            return altDrive;
        }
    }

    public double visionXDrive(double altDrive, double distance) {
        if (getRange().isPresent() && hasTargets()) {
            return driveControllerY.calculate(getRange().get() - distance);
        } else {
            return altDrive;
        }
    }


    public Optional<Double> getYaw() {
        if (hasTargets()) {
            return Optional.of(result.getBestTarget().getYaw());
        } else {
            return Optional.empty();
        }
    }

    public Optional<Double> getPitch() {
        if (hasTargets()) {
            return Optional.of(result.getBestTarget().getPitch());
        } else {
            return Optional.empty();
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
        if (!result.hasTargets()) return;

        List<PhotonTrackedTarget> evilTargets = new ArrayList<>();
        for (PhotonTrackedTarget target : result.getTargets()) {
            var tagPose = layout.getTagPose(target.getFiducialId());
            if (tagPose.isEmpty()) continue;
            var distance = PhotonUtils.getDistanceToPose(drivetrain.getState().Pose, tagPose.get().toPose2d());
            if (target.getPoseAmbiguity() > .2 || distance > VisionConstants.MAX_DISTANCE_METERS) {
                evilTargets.add(target);
            }
        }

        result.targets.removeAll(evilTargets);

        if (!result.hasTargets()) return;

        Optional<EstimatedRobotPose> estimatedPose = photonPoseEstimator.update(result);
        if (estimatedPose.isEmpty()) return;

        drivetrain.addVisionMeasurement(estimatedPose.get().estimatedPose.toPose2d(), estimatedPose.get().timestampSeconds);
    }

}