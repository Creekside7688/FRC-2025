package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;

import java.util.List;
import java.util.Optional;

import frc.robot.subsystems.SwerveDrive;

import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Limelight extends SubsystemBase {
    private final PhotonCamera robotcamera;
    private final AprilTagFieldLayout aprilTagFieldLayout;

    private List<PhotonPipelineResult> results;
    private PhotonPipelineResult latestPipeline = null;

    private PhotonTrackedTarget bestTarget;

    private PhotonPoseEstimator photonPoseEstimator;

    private final SwerveDrive sd;

    public Limelight(SwerveDrive sd) {
        robotcamera = new PhotonCamera("7688Camera");
        this.sd = sd;

        SmartDashboard.putNumber("Desired value", 0);
        SmartDashboard.putNumber("Best Target Yaw", getBestYaw());
        SmartDashboard.putNumber("Best Target Pitch", getBestPitch());
        SmartDashboard.putNumber("Best distance", getBestDistance());
        SmartDashboard.putNumber("FID", getBestAprilTag());
        SmartDashboard.putBoolean("Robot Has targets?", CameraHasTarget());
        SmartDashboard.putString("Distance correct? ", "Init");
        SmartDashboard.putString("Pitch correct?", "Init");

        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.ROBOT_TO_CAM);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // photonPoseEstimator.setLastPose(sd.getPose());

        // PortForwarder.add(5800, "photonvision.local", 5800);
    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Best Target Yaw", getBestYaw());

        SmartDashboard.putNumber("Best Target Pitch", getBestPitch());

        SmartDashboard.putNumber("Best distance", getBestDistance());

        SmartDashboard.putNumber("FID", getBestAprilTag());

        SmartDashboard.putBoolean("Robot Has targets?", CameraHasTarget());

        SmartDashboard.putString("Distance correct? ", "Init");

        SmartDashboard.putString("Yaw correct? ", "Init");
    }

    public EstimatedRobotPose getEstimatedGlobalPose() {
        // photonPoseEstimator.setLastPose(sd.getPose());
        Optional<EstimatedRobotPose> update = photonPoseEstimator.update(latestPipeline);
        if (update.isPresent()) {
            return update.get();
        }

        return null;
    }

    public double getBestYaw() {
        if (CameraHasTarget())
            return bestTarget.getYaw();
        return 0;
    }

    public double getBestPitch() {
        if (CameraHasTarget())
            return bestTarget.getPitch();
        return 0;
    }

    public int getBestAprilTag() {
        if (CameraHasTarget())
            return bestTarget.getFiducialId();
        return -1;
    }

    public boolean CameraHasTarget() {
        return bestTarget != null;
    }

    // public boolean isAlignedRotation() {
    // double yaw = getBestYaw();
    // // Tolerance of 2 degrees
    // if (yaw > 2.00) {
    // SmartDashboard.putString("Angle lined up?", "rotate left");
    // } else if (yaw < -2.00) {
    // SmartDashboard.putString("Angle lined up?", "rotate right");
    // } else {
    // SmartDashboard.putString("Angle lined up?", CameraHasTarget() ? "Lined up" :
    // "No target");
    // return true;
    // }
    //
    // return false;
    // }

    // public boolean isAlignedPosition(double desired) {
    // double distance = getBestDistance();
    // double difference = distance - desired;
    // // Tolerance of 0.05 meters
    // if (difference > 0.05) {
    // SmartDashboard.putString("Distance correct? ", "Move closer");
    // } else if (difference > -0.05) {
    // SmartDashboard.putString("Distance correct? ", "Move away");
    // } else {
    // SmartDashboard.putString("Distance correct? ", CameraHasTarget() ? "Correct"
    // : "No target");
    // return true;
    // }

    // return false;
    // }

    public double getBestDistance() {
        return PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.CAMERA_HEIGHT,
                VisionConstants.TARGET_HEIGHT, VisionConstants.CAMERA_PITCH, getBestYaw());
    }

//    public PhotonPipelineResult getLatestResult() {
//        if (results.size() > 0)
//            return results.get(results.size() - 1);
//        return new PhotonPipelineResult();
//    }

public PhotonPipelineResult getLatestResult() {
    return robotcamera.getLatestResult();
}

    // @Override
    // public void periodic() {
    // double desired = Units.inchesToMeters(SmartDashboard.getNumber("Desired
    // value", 0));
    // // Update results and target
    // results = robotcamera.getAllUnreadResults();
    // if (results.size() > 0) {
    // latestPipeline = results.get(results.size() - 1);
    // bestTarget = latestPipeline.getBestTarget();
    // if (bestTarget != null) {
    // if (isAlignedRotation() &&
    // isAlignedPosition(desired)) {
    // robotcamera.setLED(VisionLEDMode.kOn);
    // } else {
    // robotcamera.setLED(VisionLEDMode.kOff);
    // }
    // }
    // if (bestTarget != null) {
    // EstimatedRobotPose estimatedRobotPose = getEstimatedGlobalPose();
    // if (estimatedRobotPose != null) {
    // sd.updatePoseEstimates(estimatedRobotPose);
    // }
    // }
    //
    // }
    // updateSmartDashboard();
    // }

    private boolean swap;
    @Override
    public void periodic() {
        /* 
        var latestResult = robotcamera.getLatestResult();
        var bestTarget = latestResult.getBestTarget();

        if (bestTarget != null) {
            swap = true;
            var distance = bestTarget.getBestCameraToTarget();
            var distX = Units.metersToInches(distance.getX());
            if (distX > 40) {
                sd.drive(0.8, 0.0, 0, true, false, true);
            } else if (distX < 40) {
                sd.drive(0.7, 0.0, 0, true, false, true);
            } else {
                sd.drive(0, 0, 0, true, false, true);
            }
        } else {
            if (swap == true)
                sd.drive(0, 0, 0, true, false, true);
                swap = false;
        }
                */
    }
}
