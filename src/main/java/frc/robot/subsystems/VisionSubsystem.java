// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {

    PhotonCamera camera;
    PhotonPoseEstimator photonPoseEstimator;
    static PhotonPipelineResult result;

    private final PIDController rotController = new PIDController(VisionConstants.V_Kp, VisionConstants.V_Ki, VisionConstants.V_Kd);
    
    AprilTagFieldLayout aprilTagFieldLayout =
            AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
    
    public static final Transform3d CAMERA_TO_ROBOT =
                new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, 0, 0));
    
    public VisionSubsystem() {
        camera = new PhotonCamera(VisionConstants.TARGET_CAMERA);
        photonPoseEstimator =
                new PhotonPoseEstimator(
                        aprilTagFieldLayout,
                        PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                        CAMERA_TO_ROBOT);
        result = camera.getLatestResult();

        SmartDashboard.putData("VisPID", rotController);
    }
    
      @Override
      public void periodic() {
        result = camera.getLatestResult();
        // This method will be called once per scheduler 
        
        //Shuffleboard.getTab("Vision").addBoolean("Has Tag", () -> result.hasTargets());
        //Shuffleboard.getTab("vision").addDouble("Yaw To Target", () -> getYaw().get());
        //SmartDashboard.putNumber("pidVis", visionTargetPIDCalc(RobotContainer.joystick.getZ(), ))
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
    
      public Optional<Double> getYaw() {
         if (hasTarget()) {
              return Optional.of(result.getBestTarget().getYaw());
          } else {
              return Optional.empty();
          }
      }
    
      public boolean hasTarget(){
        return result.hasTargets();
  }
}
