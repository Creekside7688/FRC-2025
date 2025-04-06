package frc.robot.commands;

import java.util.Optional;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.constants.VisionConstants;

/**
 * AutoAlign
 */
public class AutoAlign extends Command {
    // Constraints to limit profiled controllers
    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(
            VisionConstants.TRANSLATE_CONTROLLER.kV_MAX, VisionConstants.TRANSLATE_CONTROLLER.kA_MAX);

    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(
            VisionConstants.TRANSLATE_CONTROLLER.kV_MAX, VisionConstants.TRANSLATE_CONTROLLER.kA_MAX);

    private static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(
            VisionConstants.THETA_CONTROLLER.kV_MAX, VisionConstants.THETA_CONTROLLER.kA_MAX);

    private final Limelight limelight;

    private final SwerveDrive sd;

    private static final ProfiledPIDController xController = new ProfiledPIDController(
            VisionConstants.TRANSLATE_CONTROLLER.kP,
            VisionConstants.TRANSLATE_CONTROLLER.kI,
            VisionConstants.TRANSLATE_CONTROLLER.kD, X_CONSTRAINTS);

    private static final ProfiledPIDController yController = new ProfiledPIDController(
            VisionConstants.TRANSLATE_CONTROLLER.kP,
            VisionConstants.TRANSLATE_CONTROLLER.kI,
            VisionConstants.TRANSLATE_CONTROLLER.kD, Y_CONSTRAINTS);

    private static final ProfiledPIDController thetaController = new ProfiledPIDController(
            VisionConstants.THETA_CONTROLLER.kP,
            VisionConstants.THETA_CONTROLLER.kI,
            VisionConstants.THETA_CONTROLLER.kD, THETA_CONSTRAINTS);

    private static final Transform3d TAG_TO_GOAL = new Transform3d(
            new Translation3d(1.5, 0.0, 0.0),
            new Rotation3d(0.0, 0.0, Math.PI));

    private PhotonTrackedTarget lastTarget;

    private final int TAG;

    public AutoAlign(final SwerveDrive sd, final Limelight limelight, final int tag) {
        xController.setTolerance(VisionConstants.TRANSLATE_CONTROLLER.TOLERANCE);
        yController.setTolerance(VisionConstants.TRANSLATE_CONTROLLER.TOLERANCE);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.TAG = tag;

        this.limelight = limelight;
        this.sd = sd;
        addRequirements(limelight, sd);
    }

    @Override
    public void initialize() {
        lastTarget = null;
        Pose2d robotPose = sd.getPose();
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
        thetaController.reset(robotPose.getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d robotPose2d = sd.getPose();
        Pose3d robotPose = new Pose3d(robotPose2d.getX(),
                robotPose2d.getY(),
                0.0,
                new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

        PhotonPipelineResult result = limelight.getLatestResult();
        if (result.hasTargets()) {
            Optional<PhotonTrackedTarget> potentialTarget = result.getTargets().stream()
                    .filter(t -> t.getFiducialId() == TAG)
                    .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= 0.2 && t.getPoseAmbiguity() != -1)
                    .findFirst();

            if (potentialTarget.isPresent()) {
                final var target = potentialTarget.get();
                lastTarget = target;

                Pose3d cameraPose = robotPose.transformBy(new Transform3d());

                Transform3d camToTarget = Limelight.correctToMeters(target.getBestCameraToTarget());
                Pose3d targetPose = cameraPose.transformBy(camToTarget);

                Pose2d goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

                xController.setGoal(goalPose.getX());
                yController.setGoal(goalPose.getY());
                thetaController.setGoal(goalPose.getRotation().getRadians());
            }

            if (lastTarget == null)
                sd.drive(0, 0, 0, false, true, false);
            else {
                double xSpeed = xController.calculate(robotPose.getX());
                if (xController.atGoal())
                    xSpeed = 0;

                double ySpeed = yController.calculate(robotPose.getY());
                if (yController.atGoal())
                    ySpeed = 0;

                double thetaSpeed = thetaController.calculate(robotPose2d.getRotation().getRadians());
                if (thetaController.atGoal())
                    thetaSpeed = 0;
                sd.drive(xSpeed, ySpeed, thetaSpeed, false, true, false);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        sd.drive(0, 0, 0, false, true, false);
    }

    @Override
    public boolean isFinished() {
        return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
    }

}
