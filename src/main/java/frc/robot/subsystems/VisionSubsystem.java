// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {

    public PhotonCamera camera;
    PhotonPoseEstimator photonPoseEstimator;
    public static PhotonPipelineResult result;
    List<PhotonPipelineResult> allResults;
    double targetYaw, targetRange;
    boolean targetVisible;

    private final PIDController rotController = new PIDController(VisionConstants.V_Kp, VisionConstants.V_Ki, VisionConstants.V_Kd);
    
    AprilTagFieldLayout aprilTagFieldLayout =
            AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
    
    public static final Transform3d CAMERA_TO_ROBOT =
                new Transform3d(new Translation3d(0.31, 0.0, 0.248), new Rotation3d(0, 0, 0));
    
    public VisionSubsystem() {
        SmartDashboard.putData(rotController);
        camera = new PhotonCamera(VisionConstants.TARGET_CAMERA);
        photonPoseEstimator =
                new PhotonPoseEstimator(
                        aprilTagFieldLayout,
                        PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                        CAMERA_TO_ROBOT);
        result = camera.getLatestResult();

        //Shuffleboard.getTab("Teleop").add(rotController);
    }

    public Optional<Double> getRange() {
        if (hasTarget()) {
            // Get the range to the target using the best target's pose
            return Optional.of(result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm());
        }
        return Optional.empty();
    }
    
      @Override
      public void periodic() {
        result = camera.getLatestResult();
        allResults = camera.getAllUnreadResults();

        /* double tagHeight = aprilTagFieldLayout.getTagPose(result.getBestTarget().getFiducialId()).get().getZ();
        
        double driveX = PhotonUtils.calculateDistanceToTargetMeters(
            0.248, // Measured with a tape measure, or in CAD.
            tagHeight, // From 2024 game manual for ID 7
            Units.degreesToRadians(0), // Measured with a protractor, or in CAD.
            Units.degreesToRadians(getPitch().get())); */
        
        //SmartDashboard.putNumber("drive-x", driveX);
        //Shuffleboard.getTab("Vision").addBoolean("Has Tag", () -> result.hasTargets());
        //Shuffleboard.getTab("vision").addDouble("Yaw To Target", () -> getYaw().get());
        //SmartDashboard.putNumber("pidVis", visionTargetPIDCalc(RobotContainer.joystick.getZ(), ))
        //SmartDashboard.putNumber("Yaw", getYaw().get());
        //SmartDashboard.putNumber("Pitch", getPitch().get());
      }

      PIDController rotPidController =
            new PIDController(VisionConstants.V_Kp, VisionConstants.V_Ki, VisionConstants.V_Kd);

      public double visionTargetPIDCalc(
         double altRotation, boolean visionMode) {
         boolean target = hasTarget();
         Optional<Double> yaw = getYaw();

         if (target && visionMode && yaw.isPresent()) {
             return -rotPidController.calculate(yaw.get());
         }
         if ((visionMode == true) && !target) {
             return altRotation;
         }
          return altRotation;
      }

      public double[] drivePid(double x, double y, double t, boolean mode){
        if (!allResults.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = allResults.get(allResults.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == 7) {
                        // Found Tag 7, record its information
                        targetYaw = target.getYaw();
                        targetRange =
                                PhotonUtils.calculateDistanceToTargetMeters(
                                        0.5, // Measured with a tape measure, or in CAD.
                                        1.435, // From 2024 game manual for ID 7
                                        Units.degreesToRadians(0), // Measured with a protractor, or in CAD.
                                        Units.degreesToRadians(target.getPitch()));
                        targetVisible = true;
                    }
                }
            }
        }
        // Auto-align when requested
        if (mode && targetVisible) {
            // Driver wants auto-alignment to tag 7
            // And, tag 7 is in sight, so we can turn toward it.
            // Override the driver's turn and fwd/rev command with an automatic one
            // That turns toward the tag, and gets the range right.
            t =
                    (0 - targetYaw) * 0.05;
            x =
                    (2 - targetRange) * 0.005;
        }
        double drive[] = {x, y, t};
                return drive;
      }
    
      public Optional<Double> getYaw() {
         if (hasTarget()) {
              return Optional.of(result.getBestTarget().getYaw());
          } else {
              return Optional.of(1000.0);
          }
      }

      public Optional<Double> getPitch() {
        if (hasTarget()) {
             return Optional.of(result.getBestTarget().getPitch());
         } else {
             return Optional.of(10000.0);
         }
     }
    
      public boolean hasTarget(){
        return result.hasTargets();
  }
}
