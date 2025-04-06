package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFields;
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

    private Pose2d currentPose;

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
        final var robotPose = sd.getPose();
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
        thetaController.reset(robotPose.getRotation().getRadians());

        currentPose = sd.getPose();
    }

    @Override
    public void execute() {
        final var robotPose2d = sd.getPose();
        final var robotPose = new Pose3d(robotPose2d.getX(),
                robotPose2d.getY(),
                0.0,
                new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

        final var result = limelight.getLatestResult();
        if (result.hasTargets()) {
            final var potentialTarget = result.getTargets().stream()
                    .filter(t -> t.getFiducialId() == TAG)
                    .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= 0.2 && t.getPoseAmbiguity() != -1)
                    .findFirst();

            if (potentialTarget.isPresent()) {
                final var target = potentialTarget.get();
                lastTarget = target;

                final var cameraPose = robotPose.transformBy(new Transform3d());

                final var camToTarget = target.getBestCameraToTarget();
                final var targetPose = cameraPose.transformBy(camToTarget);

                final var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

                xController.setGoal(goalPose.getX());
                yController.setGoal(goalPose.getY());
                thetaController.setGoal(goalPose.getRotation().getRadians());
            }

            if (lastTarget == null)
                sd.drive(0, 0, 0, false, true, false);
            else {
                var xSpeed = xController.calculate(robotPose.getX());
                if (xController.atGoal())
                    xSpeed = 0;

                var ySpeed = yController.calculate(robotPose.getY());
                if (yController.atGoal())
                    ySpeed = 0;

                var thetaSpeed = thetaController.calculate(robotPose2d.getRotation().getRadians());
                if (thetaController.atGoal())
                    thetaSpeed = 0;
                sd.drive(xSpeed, ySpeed, thetaSpeed, false, true, false);
            }
        }
    }

    @Override
    public void end(final boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }

}
