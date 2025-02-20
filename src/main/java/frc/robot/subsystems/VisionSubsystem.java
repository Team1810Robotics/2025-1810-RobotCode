package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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

    public PIDController rotController = new PIDController(VisionConstants.VR_Kp, VisionConstants.VR_Ki, VisionConstants.VR_Kd);
    public PIDController driveControllerY = new PIDController(VisionConstants.VY_Kp,VisionConstants.VY_Ki,VisionConstants.VY_Kd);
    public PIDController driveControllerX = new PIDController(VisionConstants.VX_Kp,VisionConstants.VX_Ki,VisionConstants.VX_Kd);

    
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField();
    
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

        Shuffleboard.getTab("Vision").add(rotController);
        Shuffleboard.getTab("Vision").add(driveControllerY);
        Shuffleboard.getTab("Vision").add(driveControllerX);

        Shuffleboard.getTab("Vision").addNumber("Yaw", () -> getYaw().get());
        Shuffleboard.getTab("Vision").addNumber("Pitch", () -> getPitch().get());
        Shuffleboard.getTab("Vision").addNumber("Range", () -> getRange().get());

        Shuffleboard.getTab("Vision").addNumber("Distance Error", () -> getRangeError().get());

        Shuffleboard.getTab("Vision").addNumber("driveYOut", () -> visionDrive(0, 0.5, getRange().get(), true, driveControllerY));
        Shuffleboard.getTab("Vision").addNumber("driveXOut", () -> visionDrive(0, 0.0, getXRange().get(), true, driveControllerX));
    }

    public Optional<Double> getRangeError(){
        if (hasTarget()) {
            return Optional.of(getRange().get() - 0.5);
        }
        else return Optional.of(1000.0);
    }

    public Optional<Double> getRange() {
        if (hasTarget()) {
            // Get the range to the target using the best target's pose
            return Optional.of(result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm());
        }
        return Optional.of(1000.0);
    }

    public Optional<Double> getXRange(){
        if (hasTarget()){
            return Optional.of(result.getBestTarget().altCameraToTarget.getX());
        } else return Optional.of(1000.0);
    }
    
      @Override
      public void periodic() {
        result = camera.getLatestResult();
        allResults = camera.getAllUnreadResults();
      }

      public double visionTargetPIDCalc(double altRotation, boolean visionMode) {
         boolean target = hasTarget();
         Optional<Double> yaw = getYaw();

         if (target && visionMode && yaw.isPresent()) {
             return -rotController.calculate(yaw.get());
         }
         if ((visionMode == true) && !target) {
             return altRotation;
         }
          return altRotation;
      }

      /**
       * Used to make the robot drive to a set distance away from an apriltag.
       * 
       * @param altDrive The input from the controller for then no tag is present.
       * @param distance How far away from the apriltag do we want to be.
       * @param driveMode True/False input for if we do want to drive towards the apriltag.
       * @return
       */
      public double visionDrive(double altDrive, double distance, double source, boolean driveMode, PIDController pid){
        if(getRange().isPresent() && driveMode && hasTarget()){
            return pid.calculate(source - distance);
        } else return altDrive;
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
             return Optional.of(1000.0);
         }
     }
    
      public boolean hasTarget(){
        return result.hasTargets();
  }
}
