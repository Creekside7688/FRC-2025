package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
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
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Limelight extends SubsystemBase {
    private final PhotonCamera robotcamera;
    private final AprilTagFieldLayout aprilTagFieldLayout;

    private List<PhotonPipelineResult> results;
    private PhotonPipelineResult latestPipeline = null;

    private PhotonTrackedTarget target;
    private PhotonPoseEstimator photonPoseEstimator;

    private final Transform3d robotToCam = new Transform3d(new Translation3d(VisionConstants.ROBOT_CAMERA_OFFSET_FWD,
            0.0, VisionConstants.ROBOT_CAMERA_OFFSET_VERT), new Rotation3d(0, 0, 0));

    private final Transform3d camToRobot = new Transform3d(
            new Translation3d(-VisionConstants.ROBOT_CAMERA_OFFSET_FWD, 0.0,
                    -VisionConstants.ROBOT_CAMERA_OFFSET_VERT),
            new Rotation3d(0, 0, 0));

    private final SwerveDrive sd;

    public Limelight(SwerveDrive sd) {
        robotcamera = new PhotonCamera("7688Camera");
        this.sd = sd;

        SmartDashboard.putNumber("Target Yaw", 0);
        SmartDashboard.putNumber("Target Pitch", 0);
        SmartDashboard.putNumber("Target FID", 0);
        SmartDashboard.putBoolean("Robot Has targets?", false);

        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToCam);

        photonPoseEstimator.setLastPose(sd.getPose());

        // PortForwarder.add(5800, "photonvision.local", 5800);
    }

    // public void getRobotPos() {
    // if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
    // Pose3d robotPose =
    // PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
    // aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), camToRobot);
    // }
    // }

    public double correctToMeters(double x) {
        return x * 0.1514;
    }

    public Pose2d correctToMeters(Pose2d x) {
        return x.times(0.1514);
    }

    public Pose3d correctToMeters(Pose3d x) {
        return x.times(0.1514);
    }

    public void updateSD() {
        SmartDashboard.putNumber("Best Target Yaw", getBestYaw());

        SmartDashboard.putNumber("Best Target Pitch", getBestPitch());

        SmartDashboard.putNumber("FID", getBestAprilTag());

        SmartDashboard.putBoolean("Robot Has targets?", CameraHasTargets());
    }

    public Pose3d getEstimatedRobotPosition() {
        if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
            return correctToMeters(PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
                    aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), camToRobot));
        }

        return null;
    }

    public EstimatedRobotPose getEstimatedGlobalPose() {
        var update = photonPoseEstimator.update(latestPipeline);
        if (update.isPresent()) {
            var out = update.get();
            return new EstimatedRobotPose(correctToMeters(out.estimatedPose), out.timestampSeconds, out.targetsUsed,
                    out.strategy);
        }

        return null;
    }

    public double getBestYaw() {
        if (target != null) {
            return target.getYaw();
        }

        return 0;
    }

    public double getBestPitch() {
        if (target != null) {
            return target.getPitch();
        }
        return 0;
    }

    public int getBestAprilTag() {
        if (target != null)
            return target.getFiducialId();

        return -1;
    }

    public boolean CameraHasTargets() {
        return target != null;
    }

    public void alignRotation() {
        double yaw = getBestYaw();
        // Tolerance of 5 degree
        if (yaw > 5.00) {
            SmartDashboard.putString("Angle lined up?", "rotate left");
        } else if (yaw < -5.00) {
            SmartDashboard.putString("Angle lined up?", "rotate right");
        } else {
            SmartDashboard.putString("Angle lined up?", CameraHasTargets() ? "Lined up" : "No target");
        }
    }

    public void alignPosition(double desired) {
        double distance = getBestDistance();
        double difference = distance - desired;
        // Tolerance of 0.3 meters
        if (difference > 0.3) {
            SmartDashboard.putString("Distance correct? ", "Move closer");
        } else if (difference > -0.3) {
            SmartDashboard.putString("Distance correct? ", "Move away");
        } else {
            SmartDashboard.putString("Distance correct? ", CameraHasTargets() ? "Correct" : "No target");
        }
    }

    public double getBestDistance() {
        return correctToMeters(PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.CAMERA_HEIGHT,
                VisionConstants.TARGET_HEIGHT, VisionConstants.CAMERA_PITCH, getBestYaw()));
    }

    public PhotonPipelineResult getLatestResult() {
        if (results.size() > 0)
            return results.get(results.size() - 1);
        return new PhotonPipelineResult();
    }

    @Override
    public void periodic() {
        double desired = Units.inchesToMeters(SmartDashboard.getNumber("Desired value", 0));
        // Update results and target
        results = robotcamera.getAllUnreadResults();
        if (results.size() > 0) {
            // results is FIFO structure - get item most recently pushed onto stack
            // should change code to confirm if target matches desired ficudal tag ID
            latestPipeline = results.get(results.size() - 1);
            target = latestPipeline.getBestTarget();
            alignRotation();
            alignPosition(desired);

            if (target != null) {
                EstimatedRobotPose estimatedRobotPose = getEstimatedGlobalPose();
                if (estimatedRobotPose != null) {
                    sd.updatePoseEstimates(estimatedRobotPose);
                }
            }
        }

    }
}
