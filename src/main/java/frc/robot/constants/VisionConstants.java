package frc.robot.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * LimelightConstants
 */

public class VisionConstants {

    public static class TRANSLATE_CONTROLLER {
        public static final double kP = 1;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double TOLERANCE = Units.inchesToMeters(1);

        public static final double kV_MAX = 2.5;
        public static final double kA_MAX = 1.0;
    }

    public static class THETA_CONTROLLER {
        public static final double kP = 1;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double TOLERANCE = Units.degreesToRadians(3);
        public static final double kV_MAX = 0.5;
        public static final double kA_MAX = 1.0;
    }

    public static final double CAMERA_HEIGHT = 0;
    public static final double CAMERA_PITCH = 0;
	public static final double TARGET_HEIGHT = 0;

    public static final double ROBOT_CAMERA_OFFSET_FWD = 0;
    public static final double ROBOT_CAMERA_OFFSET_VERT = 0;

    public static final double CAMERA_TURN_RANGE = 0.01;
    public static final double CAMERA_TAG_RANGE = 2.0; // meters
    public static final double ROBOT_STRAFE_GAIN = 0.02;

    public static final double FIELD_LENGTH_METERS = 16.54175;
    public static final double FIELD_WIDTH_METERS = 8.0137;

    // Pose on the opposite side of the field. Use with `relativeTo` to flip a pose to the opposite alliance
    public static final Pose2d FLIPPING_POSE = new Pose2d(
        new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS),
        new Rotation2d(Math.PI)
    );

    public static final Transform3d ROBOT_TO_CAM = new Transform3d(new Translation3d(ROBOT_CAMERA_OFFSET_FWD,
            0.0, ROBOT_CAMERA_OFFSET_VERT), new Rotation3d(0, 0, 0));
}
