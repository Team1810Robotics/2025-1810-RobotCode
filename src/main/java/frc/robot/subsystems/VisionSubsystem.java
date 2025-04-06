package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

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
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {

    public PhotonCamera camera;
    PhotonPoseEstimator photonPoseEstimator;
    public static PhotonPipelineResult result;
    List<PhotonPipelineResult> allResults;

    private CommandSwerveDrivetrain drivetrain;

    public PIDController rotController = new PIDController(VisionConstants.VR_kP, VisionConstants.VR_kI,
            VisionConstants.VR_kD);
    public PIDController driveControllerY = new PIDController(VisionConstants.VY_kP,
            VisionConstants.VY_kI, VisionConstants.VY_kD);
    public PIDController driveControllerX = new PIDController(VisionConstants.VX_kP,
            VisionConstants.VX_kI, VisionConstants.VX_kD);

    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public VisionSubsystem(String cameraName, Transform3d cameraOffset, CommandSwerveDrivetrain drivetrain) {
        camera = new PhotonCamera(cameraName);

        photonPoseEstimator = new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, //TODO: Update this setting in PhotonVision
                cameraOffset);
    }


    public Optional<Double> getRange() {
        if (hasTarget()) {
            // Get the range to the target using the best target's pose
            return Optional.of(result.getBestTarget().getBestCameraToTarget().getTranslation().getX());
        }
        return Optional.empty();
    } 


    public Optional<Double> getXRange() {
        if (hasTarget()) {
            return Optional.of(result.getBestTarget().bestCameraToTarget.getY());
        } else
            return Optional.empty();
    }

    public Optional<Double> getTagRYaw() {
        if (hasTarget()) {
            return Optional.of(aprilTagFieldLayout.getTagPose(result.getBestTarget().getFiducialId()).get()
                    .getRotation().getZ());
        }
        return Optional.empty();
    }


    @Override
    public void periodic() {
        if (!camera.isConnected()) {
            DriverStation.reportWarning("Camera not connected", false);
            return;
        }

        result = getLatestResult();
        if (result.hasTargets()) return;

        List<PhotonTrackedTarget> evilTargets = new ArrayList<>();
        for (PhotonTrackedTarget target : result.getTargets()) {
            var tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
            if (tagPose.isEmpty()) continue;
            var distance = PhotonUtils.getDistanceToPose(drivetrain.getState().Pose, tagPose.get().toPose2d());
            if (target.getPoseAmbiguity() > .2 || distance > VisionConstants.MAX_DISTANCE.in(Meters)) {
                evilTargets.add(target);
            }
        }

        result.targets.removeAll(evilTargets);

        if (!result.hasTargets()) return;

        Optional<EstimatedRobotPose> estimatedPose = photonPoseEstimator.update(result);
        if (estimatedPose.isEmpty()) return;

        drivetrain.addVisionMeasurement(estimatedPose.get().estimatedPose.toPose2d(), estimatedPose.get().timestampSeconds);
    }

    //TODO: test new implementation
    public double visionTargetPIDCalc(double altRotation) {
        boolean target = hasTarget();

        if (target && getYaw().isPresent()) {
            return -rotController.calculate(aprilTagFieldLayout.getTagPose(result.getBestTarget().getFiducialId())
                    .get().getRotation().getZ());
        }
 
        return altRotation;
    }

    public double visionPara(double altRotation, boolean visionMode, double gyro) {
        if (visionMode) {
            if (hasTarget()) {
                return rotController.calculate(gyro % 360
                        - Math.toDegrees(aprilTagFieldLayout.getTagPose(result.getBestTarget().getFiducialId())
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
        if (getRange().isPresent() && hasTarget()) {
            return driveControllerX.calculate(getYaw().get() - distance);
        } else {
            return altDrive;
        }
    }

    public double visionXDrive(double altDrive, double distance) {
        if (getRange().isPresent() && hasTarget()) {
            return driveControllerY.calculate(getRange().get() - distance);
        } else {
            return altDrive;
        }
    }


    public Optional<Double> getYaw() {
        if (hasTarget()) {
            return Optional.of(result.getBestTarget().getYaw());
        } else {
            return Optional.empty();
        }
    }



    public Optional<Double> getPitch() {
        if (hasTarget()) {
            return Optional.of(result.getBestTarget().getPitch());
        } else {
            return Optional.empty();
        }
    }

    public boolean hasTarget() {
        return result.hasTargets();
    }

    public PhotonPipelineResult getLatestResult() {
        allResults = camera.getAllUnreadResults();
        int size = allResults.size();
        return size > 0 ? allResults.get(size - 1) : new PhotonPipelineResult();
    }

}