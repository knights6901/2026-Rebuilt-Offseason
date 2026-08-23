package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

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

    /*
     * Standard deviation model. The XY standard deviation handed to the pose
     * estimator grows with the square of the tag distance and shrinks with the
     * number of tags used:
     *
     * xyStdDev = base * (1 + avgTagDist^2 / kDistanceDivisor) / numTags
     *
     * These are starting values and need to be tuned on the field against the
     * Vision/Residual log key.
     */
    public static final double kSingleTagXYStdDevBase = 0.15;
    public static final double kMultiTagXYStdDevBase = 0.06;
    public static final double kDistanceDivisor = 30.0;

    /**
     * Standard deviation reported for the heading component of every vision
     * measurement. Vision never corrects heading -- the gyro is treated as truth --
     * so this is made large enough that the Kalman gain for theta is effectively
     * zero. Large but finite, to avoid overflow inside the filter.
     */
    public static final double kThetaStdDev = 1e6;

    /** Discard a pipeline result older than this many seconds. */
    public static final double kMaxResultAgeSeconds = 0.2;

    /** Discard an estimate whose average camera-to-tag distance exceeds this. */
    public static final double kMaxTagDistanceMeters = 5.0;

    /**
     * Discard a 3D estimate that puts the robot this far above or below the floor.
     */
    public static final double kMaxZErrorMeters = 0.30;

    /** Discard a 3D estimate with more roll or pitch than this. */
    public static final double kMaxTiltRadians = Math.toRadians(10);

    /**
     * Discard estimates taken while spinning faster than this, since rolling
     * shutter smear corrupts the tag corners.
     */
    public static final double kMaxAngularRateRadPerSec = Math.toRadians(360);

    /** How far outside the field an estimate may land before it is discarded. */
    public static final double kFieldBorderMarginMeters = 0.30;

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
