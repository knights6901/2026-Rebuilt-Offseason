package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import static edu.wpi.first.units.Units.Inches;

/**
 * Vision processing constants for AprilTag-based localization.
 */
public final class VisionConstants {
    /** The name/identifier of the camera used for vision processing. */
    public static final String arducamName = "photonCam";

    /**
     * The 3D transformation from the robot center to the camera position and
     * orientation.
     */
    public static final Transform3d kRobotToCam = new Transform3d(
            new Translation3d(Inches.of(-13), Inches.of(0.0), Inches.of(6.5)),
            new Rotation3d(0, -Math.PI / 6, Math.PI));

    /** The layout of AprilTags on the field for localization. */
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.kDefaultField);

    // values need to be heavily tested and tuned
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    /** Discard a pipeline result older than this many seconds. */
    public static final double kMaxResultAgeSeconds = 0.2;

    /** Above this ambiguity, a single-tag pose estimate is discarded. */
    public static final double kMaxSingleTagAmbiguity = 0.1;

    /** Simulated camera resolution, in pixels. */
    public static final int kSimCameraWidthPx = 640;
    public static final int kSimCameraHeightPx = 480;

    /** Simulated camera diagonal field of view. */
    public static final Rotation2d kSimCameraFov = Rotation2d.fromDegrees(100);

    /** Simulated pixel noise: average error and standard deviation, in pixels. */
    public static final double kSimCalibErrorAvgPx = 0.05;
    public static final double kSimCalibErrorStdDevPx = 0.05;

    public static final double kSimFps = 60;
    public static final double kSimAvgLatencyMs = 20;
    public static final double kSimLatencyStdDevMs = 5;
}
