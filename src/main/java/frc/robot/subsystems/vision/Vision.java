package frc.robot.subsystems.vision;

import frc.robot.Constants.GameConstants;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;

import java.util.List;
import java.util.ArrayList;
import java.util.Optional;
import java.util.Set;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem responsible for AprilTag-based pose estimation using PhotonVision.
 *
 * <p>
 * Fuses vision measurements into the drivetrain's Kalman-filter odometry,
 * tracks visible tag IDs and poses, and publishes tag data to NetworkTables.
 * Supports full camera simulation when running in sim.
 *
 * <p>
 * Vision only ever corrects the <i>translation</i> of the drivetrain pose. The
 * gyro is treated as the sole source of truth for heading, because CTRE's
 * field-centric swerve requests take "forward" from the fused pose's rotation
 * --
 * letting vision rotate the pose would move the driver's forward direction out
 * from under them mid-match.
 */
public class Vision extends SubsystemBase {
    private final PhotonCamera photonCam;
    private final Drive drivetrain;
    private final PhotonPoseEstimator visionPoseEstimator;

    /** Camera simulation, or {@code null} when running on a real robot. */
    private VisionSim visionSim;

    private final List<Pose3d> visibleTagPoses = new ArrayList<>();
    private final List<Integer> visibleTagIds = new ArrayList<>();

    public final Field2d visionField = new Field2d();

    /**
     * The most recently estimated robot pose from vision (empty if none this
     * cycle).
     */
    private Optional<EstimatedRobotPose> estimatedPose = Optional.empty();

    /**
     * Robot-relative bearing to the center of our hub, from the last frame that
     * saw any of its tags, and when that frame was captured.
     */
    private Rotation2d hubBearing = Rotation2d.kZero;
    private double hubBearingTimestamp = Double.NEGATIVE_INFINITY;

    /**
     * Whether vision measurements are currently allowed to reach the drivetrain.
     * Cleared by the driver's panic button so that a bad estimate can be shut out
     * mid-match without restarting code.
     */
    private boolean fusionEnabled = true;

    /**
     * Whether the drivetrain's pose has been (re)seeded from vision since the last
     * {@link #reseedPose()} call.
     */
    public boolean hasSeededPose = DriverStation.isFMSAttached() ? true : false;

    /**
     * Creates the vision subsystem, initializing PhotonVision cameras and the
     * pose estimator from the current year's field layout. Starts camera
     * simulation if running in sim.
     */
    public Vision(Drive drivetrain) {
        this.drivetrain = drivetrain;

        photonCam = new PhotonCamera(VisionConstants.arducamName);
        visionPoseEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout, VisionConstants.kRobotToCam);

        if (Robot.isSimulation()) {
            visionSim = new VisionSim(drivetrain, photonCam);
        }

        SmartDashboard.putData("VisionField", visionField);
    }

    /**
     * Returns the 3D field poses of all AprilTags currently visible to the camera.
     *
     * @return list of visible tag poses (may be empty)
     */
    public List<Pose3d> getVisibleTagPoses() {
        return visibleTagPoses;
    }

    /**
     * Returns the fiducial IDs of all AprilTags currently visible to the camera.
     *
     * @return list of visible tag IDs (may be empty)
     */
    public List<Integer> getVisibleTagIds() {
        return visibleTagIds;
    }

    /**
     * Returns the robot-relative bearing to the center of our alliance's hub, as
     * measured directly off its AprilTags, or empty if the camera hasn't seen them
     * recently. CCW-positive, with zero straight out the front of the robot.
     *
     * <p>
     * This comes purely from the camera's view of the tags and never touches the
     * drivetrain pose, so it stays correct even when the pose estimate has drifted.
     */
    public Optional<Rotation2d> getHubBearing() {
        boolean isStale = Timer.getTimestamp() - hubBearingTimestamp > VisionConstants.Filter.kMaxResultAgeSeconds;
        return isStale ? Optional.empty() : Optional.of(hubBearing);
    }

    /**
     * Enables or disables fusion of vision measurements into the drivetrain pose.
     * Tag tracking and logging continue either way.
     */
    public void setFusionEnabled(boolean enabled) {
        fusionEnabled = enabled;
    }

    @Override
    public void periodic() {
        visionPoseEstimator.addHeadingData(Timer.getTimestamp(), drivetrain.getPose().getRotation());

        visibleTagPoses.clear();
        visibleTagIds.clear();
        estimatedPose = Optional.empty();

        /*
         * Every unread frame is fused in order, rather than only the newest. The camera
         * outruns the robot loop, so the extra frames are free measurements.
         */
        for (PhotonPipelineResult result : photonCam.getAllUnreadResults()) {
            if (!isUsable(result)) {
                continue;
            }

            recordVisibleTags(result);
            recordHubBearing(result);

            Optional<EstimatedRobotPose> estimate = estimatePose(result);
            if (estimate.isEmpty()) {
                continue;
            }

            estimatedPose = estimate;
            adjustDrivetrainPose(estimate.get());
        }

        estimatedPose.ifPresent(pose -> {
            visionField.setRobotPose(pose.estimatedPose.toPose2d());
        });
    }

    /** Whether a pipeline result has targets and is fresh enough to trust. */
    private boolean isUsable(PhotonPipelineResult result) {
        boolean isStale = Timer.getTimestamp()
                - result.getTimestampSeconds() > VisionConstants.Filter.kMaxResultAgeSeconds;
        return result.hasTargets() && !isStale;
    }

    /** Replaces the visible-tag lists with the tags seen in this result. */
    private void recordVisibleTags(PhotonPipelineResult result) {
        visibleTagPoses.clear();
        visibleTagIds.clear();

        for (PhotonTrackedTarget target : result.getTargets()) {
            VisionConstants.kTagLayout.getTagPose(target.getFiducialId()).ifPresent(visibleTagPoses::add);
            visibleTagIds.add(target.getFiducialId());
        }
    }

    /**
     * Updates the hub bearing from the hub tags in this result, if there are any.
     *
     * <p>
     * Each tag sits off to one side of the hub, and the camera sits off to one
     * side of the robot, so a tag's raw yaw isn't the angle we want. Instead each
     * tag's camera-relative transform is chained with where the camera is on the
     * robot and where the hub center is relative to that tag, which places the hub
     * center in the robot's own frame. The estimates from every tag are averaged.
     */
    private void recordHubBearing(PhotonPipelineResult result) {
        Translation2d hubLocation = GameConstants.getHubLocation();
        Pose3d hub = new Pose3d(hubLocation.getX(), hubLocation.getY(), 0, Rotation3d.kZero);
        Set<Integer> hubTagIds = GameConstants.getHubTagIds();

        Translation2d robotToHub = Translation2d.kZero;
        boolean sawHub = false;

        for (PhotonTrackedTarget target : result.getTargets()) {
            Optional<Pose3d> tagPose = VisionConstants.kTagLayout.getTagPose(target.getFiducialId());

            if (!hubTagIds.contains(target.getFiducialId())
                    || tagPose.isEmpty()
                    || target.getPoseAmbiguity() > VisionConstants.Filter.kMaxPoseAmbiguity) {
                continue;
            }

            Pose3d tagInRobotFrame = Pose3d.kZero
                    .transformBy(VisionConstants.kRobotToCam)
                    .transformBy(target.getBestCameraToTarget());
            Pose3d hubInRobotFrame = tagInRobotFrame.transformBy(new Transform3d(tagPose.get(), hub));

            robotToHub = robotToHub.plus(hubInRobotFrame.getTranslation().toTranslation2d());
            sawHub = true;
        }

        if (!sawHub) {
            return;
        }

        /* The sum points the same way as the average, so there's no need to divide. */
        hubBearing = robotToHub.getAngle();
        hubBearingTimestamp = result.getTimestampSeconds();
    }

    /**
     * Estimates a robot pose from a fresh, non-empty pipeline result.
     *
     * <p>
     * With two or more tags the coprocessor's multi-tag PnP solution is used. With
     * a single tag we use the trig solve instead of a PnP solve: it derives
     * translation from the tag's measured distance and bearing combined with the
     * gyro heading, so there is no orientation solve and therefore no pose
     * ambiguity to flip between frames.
     */
    private Optional<EstimatedRobotPose> estimatePose(PhotonPipelineResult result) {
        if (result.getTargets().size() > 1) {
            return visionPoseEstimator.estimateCoprocMultiTagPose(result);
        }

        return visionPoseEstimator.estimatePnpDistanceTrigSolvePose(result);
    }

    public Optional<Pose2d> getEstimatedPose2d() {
        return estimatedPose.map(pose -> pose.estimatedPose.toPose2d());
    }

    /**
     * Sanity-checks a vision estimate and, if it passes, feeds it to the
     * drivetrain's pose estimator with a distance- and tag-count-scaled weight.
     */
    private void adjustDrivetrainPose(EstimatedRobotPose estimate) {
        Pose2d pose2d = estimate.estimatedPose.toPose2d();
        int numTags = estimate.targetsUsed.size();
        double avgTagDistance = averageTagDistance(estimate);

        String rejection = rejectionReason(estimate, avgTagDistance, numTags);

        if (rejection != null) {
            return;
        }

        /*
         * A reseed is a deliberate request to snap to where vision says we are, so it
         * bypasses the filter -- but only the translation, never the heading.
         */
        if (!hasSeededPose) {
            hasSeededPose = true;
            drivetrain.resetTranslation(pose2d.getTranslation());
            return;
        }

        /*
         * Trust falls off with the square of tag distance and improves with the number
         * of tags in the solution.
         */
        double base = numTags > 1
                ? VisionConstants.StdDevs.kMultiTagXYBase
                : VisionConstants.StdDevs.kSingleTagXYBase;
        double xyStdDev = base
                * (1 + Math.pow(avgTagDistance, 2) / VisionConstants.StdDevs.kDistanceDivisor)
                / numTags;

        /*
         * Heading is taken from the drivetrain rather than from vision, and paired with
         * an enormous theta standard deviation. Either alone would keep vision from
         * rotating the pose; together they make it structurally impossible, which is
         * what keeps field-oriented driving identical to gyro-only behaviour.
         */
        Pose2d measurement = new Pose2d(pose2d.getTranslation(), drivetrain.getPose().getRotation());
        Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStdDev, xyStdDev, VisionConstants.StdDevs.kTheta);

        drivetrain.addVisionMeasurement(measurement, estimate.timestampSeconds, stdDevs);
    }

    /**
     * Returns why this estimate should be thrown out, or {@code null} if it is
     * trustworthy.
     */
    private String rejectionReason(EstimatedRobotPose estimate, double avgTagDistance, int numTags) {
        if (!fusionEnabled) {
            return "Fusion suspended";
        }

        if (numTags == 0) {
            return "No targets used";
        }

        if (numTags == 1) {
            if (estimate.targetsUsed.get(0).poseAmbiguity > VisionConstants.Filter.kMaxPoseAmbiguity) {
                return "Ambiguity too high";
            }
        }

        if (avgTagDistance > VisionConstants.Filter.kMaxTagDistanceMeters) {
            return "Tags too far";
        }

        double angularRate = Math.abs(drivetrain.getState().Speeds.omegaRadiansPerSecond);
        if (angularRate > VisionConstants.Filter.kMaxAngularRateRadPerSec) {
            return "Spinning too fast";
        }

        double linearRate = Math.abs(
                Math.sqrt(
                        Math.pow(drivetrain.getState().Speeds.vxMetersPerSecond, 2) +
                                Math.pow(drivetrain.getState().Speeds.vyMetersPerSecond, 2)));
        if (linearRate > VisionConstants.Filter.kMaxLinearRateMPerSec) {
            return "Moving too fast";
        }

        if (isOutsideField(estimate.estimatedPose.toPose2d())) {
            return "Outside field";
        }

        /*
         * The trig solve pins z, roll and pitch to zero by construction, so these
         * checks only mean something for the strategies that actually solve for a full
         * 3D pose.
         */
        if (estimate.strategy != PoseStrategy.PNP_DISTANCE_TRIG_SOLVE) {
            if (Math.abs(estimate.estimatedPose.getZ()) > VisionConstants.Filter.kMaxZErrorMeters) {
                return "Bad Z height";
            }

            Rotation3d rotation = estimate.estimatedPose.getRotation();
            if (Math.abs(rotation.getX()) > VisionConstants.Filter.kMaxTiltRadians
                    || Math.abs(rotation.getY()) > VisionConstants.Filter.kMaxTiltRadians) {
                return "Tilted";
            }
        }

        return null;
    }

    /** Whether a pose lands outside the field, allowing a small border margin. */
    private boolean isOutsideField(Pose2d pose) {
        double margin = VisionConstants.Filter.kFieldBorderMarginMeters;

        return pose.getX() < -margin
                || pose.getX() > VisionConstants.kTagLayout.getFieldLength() + margin
                || pose.getY() < -margin
                || pose.getY() > VisionConstants.kTagLayout.getFieldWidth() + margin;
    }

    /** Mean camera-to-tag distance across the targets used in an estimate. */
    private double averageTagDistance(EstimatedRobotPose estimate) {
        if (estimate.targetsUsed.isEmpty()) {
            return 0;
        }

        double total = 0;
        for (PhotonTrackedTarget target : estimate.targetsUsed) {
            total += target.getBestCameraToTarget().getTranslation().getNorm();
        }

        return total / estimate.targetsUsed.size();
    }

    public void reseedPose() {
        hasSeededPose = false;
    }

    /**
     * Teleports the simulated robot's ground-truth pose so the camera simulation
     * keeps rendering from the right place. No-op on a real robot. Call alongside
     * {@link Drive#resetPose} whenever the robot is deliberately moved in sim.
     */
    public void resetSimGroundTruth(Pose2d pose) {
        if (visionSim != null) {
            visionSim.resetGroundTruth(pose);
        }
    }

    /**
     * Resets the heading for the visionPoseEstimator in case gyro is reset; it
     * prob won't be a significant difference ngl but it could stop 1-2 frames
     * from being cooked
     */
    public void resetEstimatorHeading() {
        visionPoseEstimator.resetHeadingData(Timer.getTimestamp(), drivetrain.getPose().getRotation());
    }
}
