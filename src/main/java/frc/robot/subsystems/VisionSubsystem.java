package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {

    public PhotonCamera cameraRight, cameraLeft;
    PhotonPoseEstimator photonPoseEstimator;
    public static PhotonPipelineResult resultRight, resultLeft;
    List<PhotonPipelineResult> allResults;
    double targetYawRight, targetRangeRight;

    public PIDController rotController = new PIDController(VisionConstants.VR_Kp, VisionConstants.VR_Ki, VisionConstants.VR_Kd);
    public PIDController driveControllerY = new PIDController(VisionConstants.VY_Kp,VisionConstants.VY_Ki,VisionConstants.VY_Kd);
    public PIDController driveControllerX = new PIDController(VisionConstants.VX_Kp,VisionConstants.VX_Ki,VisionConstants.VX_Kd);

    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    
    public static final Transform3d CAMERA_TO_ROBOT_RIGHT =
        new Transform3d(new Translation3d(0.127, 0.17145, 0.3175), new Rotation3d(0, 0, 0)); //13.5 6 6.4
    
        public static final Transform3d CAMERA_TO_ROBOT_LEFT =
        new Transform3d(new Translation3d(0.127, -0.17145, 0.3175), new Rotation3d(0, 0, 0)); //13.5 6 6.4
    
    public VisionSubsystem() {
        SmartDashboard.putData(rotController);
        cameraRight = new PhotonCamera(VisionConstants.TARGET_CAMERA_RIGHT);
        cameraLeft = new PhotonCamera(VisionConstants.TARGET_CAMERA_LEFT);
        photonPoseEstimator =
                new PhotonPoseEstimator(
                        aprilTagFieldLayout,
                        PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                        CAMERA_TO_ROBOT_RIGHT);
        resultRight = cameraRight.getLatestResult();
        resultLeft = cameraLeft.getLatestResult();

        Shuffleboard.getTab("Vision").addBoolean("Has Camera Right", () -> cameraRight.isConnected());
        Shuffleboard.getTab("Vision").addBoolean("Has Camera Left", () -> cameraLeft.isConnected());

        Shuffleboard.getTab("Vision/Test").add("Vision Rotiation PID", rotController);
        Shuffleboard.getTab("Vision/Test").add("Vision Y PID", driveControllerY);
        Shuffleboard.getTab("Vision").add("Vision X PID", driveControllerX);

/*         Shuffleboard.getTab("Vision").addNumber("Yaw Left", () -> getYawLeft().get());
        Shuffleboard.getTab("Vision").addNumber("Pitch Left", () -> getPitchLeft().get());
        Shuffleboard.getTab("Vision").addNumber("Range Left", () -> getRangeLeft().get());
        Shuffleboard.getTab("Vision").addNumber("Range - X Left", () -> getXRangeLeft().get());

        Shuffleboard.getTab("Vision").addBoolean("Has Target", () -> hasTarget());
        Shuffleboard.getTab("Vision").addBoolean("Range Present Left", () -> getRangeLeft().isPresent());

        Shuffleboard.getTab("Vision").addNumber("Distance Error Left", () -> getRangeErrorLeft().get());

        Shuffleboard.getTab("Vision").addNumber("driveYOut Left", () -> visionYDriveLeft(0.0, 0.5, true, driveControllerY));
        Shuffleboard.getTab("Vision").addNumber("driveXOut Left", () -> visionXDriveLeft(0.0, 0.0, true, driveControllerX));
        
        Shuffleboard.getTab("Vision").addNumber("driveYOut Right", () -> visionYDriveRight(0.0, 0.5, true, driveControllerY));
        Shuffleboard.getTab("Vision").addNumber("driveXOut Right", () -> visionXDriveRight(0.0, 0.0, true, driveControllerX)); */
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update(resultRight);
    }

    public Optional<Double> getRangeErrorLeft(){
        if (leftHasTarget()) {
            return Optional.of(getRangeLeft().get() - 0.5);
        }
        else return Optional.of(1000.0);
    }

    public Optional<Double> getRangeErrorRight(){
        if (rightHasTarget()) {
            return Optional.of(getRangeRight().get() - 0.5);
        }
        else return Optional.of(1000.0);
    }

    public Optional<Double> getRangeLeft() {
        if (leftHasTarget()) {
            // Get the range to the target using the best target's pose
            return Optional.of(resultLeft.getBestTarget().getBestCameraToTarget().getTranslation().getX());
        }
        return Optional.of(1000.0);
    } //bennett martin is evil

    public Optional<Double> getRangeRight() {
        if (rightHasTarget()) {
            // Get the range to the target using the best target's pose
            return Optional.of(resultRight.getBestTarget().getBestCameraToTarget().getTranslation().getX());
        }
        return Optional.of(1000.0);
    } //bennett martin is evil

    public Optional<Double> getXRangeLeft(){
        if (leftHasTarget()){
            return Optional.of(resultLeft.getBestTarget().bestCameraToTarget.getY());
        } else return Optional.of(1000.0);
    }

    public Optional<Double> getXRangeRight(){
        if (rightHasTarget()){
            return Optional.of(resultRight.getBestTarget().bestCameraToTarget.getY());
        } else return Optional.of(1000.0);
    }

    public Optional<Double> getTagRYawLeft(){
        if(leftHasTarget()){
            return Optional.of(aprilTagFieldLayout.getTagPose(resultLeft.getBestTarget().getFiducialId()).get().getRotation().getZ());
        }
        return Optional.of(1000.0);
    }

    public Optional<Double> getTagRYawRight(){
        if(rightHasTarget()){
            return Optional.of(aprilTagFieldLayout.getTagPose(resultRight.getBestTarget().getFiducialId()).get().getRotation().getZ());
        }
        return Optional.of(1000.0);
    }
    
      @Override
      public void periodic() {
        resultRight = cameraRight.getLatestResult();
        resultLeft = cameraLeft.getLatestResult();
      }

      public double visionTargetPIDCalcLeft(double altRotation, boolean visionModeLeft) {
        boolean target = leftHasTarget();

        if (target && visionModeLeft && getYawLeft().isPresent()) {
            return -rotController.calculate(getYawLeft().get());
        }
        if ((visionModeLeft == true) && !target) {
            return altRotation;
        }
         return altRotation;
     }

      public double visionTargetPIDCalcRight(double altRotation, boolean visionModeRight) {
        boolean target = leftHasTarget();

        if (target && visionModeRight && getYawRight().isPresent()) {
            return -rotController.calculate(getYawRight().get());
        }
        if ((visionModeRight == true) && !target) {
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
      public double visionYDriveLeft(double altDrive, double distance, boolean driveModeLeft, PIDController pid){
        if (getRangeLeft().isPresent() && driveModeLeft && leftHasTarget()){
            return pid.calculate(getYawLeft().get() - distance);
        } else return altDrive;
      }

      public double visionYDriveRight(double altDrive, double distance, boolean driveModeRight, PIDController pid){
        if (getRangeRight().isPresent() && driveModeRight && rightHasTarget()){
            return pid.calculate(getYawRight().get() - distance);
        } else return altDrive;
      }

      public double visionXDriveLeft(double altDrive, double distance, boolean driveModeLeft, PIDController pid){
        if (getRangeLeft().isPresent() && driveModeLeft && leftHasTarget()){
            return pid.calculate(getRangeLeft().get() - distance);
        } else return altDrive;
      }

      public double visionXDriveRight(double altDrive, double distance, boolean driveModeRight, PIDController pid){
        if (getRangeRight().isPresent() && driveModeRight && rightHasTarget()){
            return pid.calculate(getRangeRight().get() - distance);
        } else return altDrive;
      }
    
      public Optional<Double> getYawLeft() {
         if (leftHasTarget()) {
              return Optional.of(resultLeft.getBestTarget().getYaw());
          } else {
              return Optional.of(1000.0);
          }
      }

      public Optional<Double> getYawRight() {
        if (rightHasTarget()) {
             return Optional.of(resultRight.getBestTarget().getYaw());
         } else {
             return Optional.of(1000.0);
         }
     }

      public Optional<Double> getPitchLeft() {
        if (leftHasTarget()) {
             return Optional.of(resultLeft.getBestTarget().getPitch());
         } else {
             return Optional.of(1000.0);
         }
     }

     public Optional<Double> getPitchRight() {
        if (rightHasTarget()) {
             return Optional.of(resultRight.getBestTarget().getPitch());
         } else {
             return Optional.of(1000.0);
         }
     }
    
    public boolean leftHasTarget(){
        return resultLeft.hasTargets();
    }

    public boolean rightHasTarget(){
        return resultRight.hasTargets();
    }
}