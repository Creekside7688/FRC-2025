package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;

public class Limelight extends SubsystemBase {
    private final PhotonCamera robotcamera;
    // private final AprilTagFieldLayout aprilTagFieldLayout;

    private List<PhotonPipelineResult> results;
    private PhotonPipelineResult latestPipeline;

    // private PhotonPoseEstimator photonPoseEstimator;

    public Limelight() {
        robotcamera = new PhotonCamera(VisionConstants.CAMERA_NAME);

        // aprilTagFieldLayout =
        // AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
        // PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.ROBOT_TO_CAM);
        // photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    // public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    // if (cameraHasTarget())
    // return photonPoseEstimator.update(latestPipeline);
    // return Optional.empty();
    // }
    //
    // public boolean cameraHasTarget() {
    // return latestPipeline.hasTargets();
    // }

    public PhotonPipelineResult getLatestResult() {
        return latestPipeline;
    }

    @Override
    public void periodic() {
        results = robotcamera.getAllUnreadResults();

        if (results.size() > 0) {
            latestPipeline = results.get(results.size() - 1);
        }
    }
}
