package frc.robot.subsystems.vision;

import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;

import java.util.List;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
 */
public class Vision extends SubsystemBase {
    private final PhotonCamera photonCam;
    private final Drive drivetrain;
    private final PhotonPoseEstimator visionPoseEstimator;

    private final List<Pose3d> visibleTagPoses = new ArrayList<>();
    private final List<Integer> visibleTagIds = new ArrayList<>();

    public final Field2d visionField = new Field2d();

    /**
     * The most recently estimated robot pose from vision (empty if none this
     * cycle).
     */
    private Optional<EstimatedRobotPose> estimatedPose = Optional.empty();
    private boolean multiTag = false;

    /**
     * Whether the drivetrain's pose has been (re)seeded from vision since the last
     * {@link #reseedPose()} call.
     */
    public boolean hasSeededPose = true;

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
            new VisionSim(drivetrain, photonCam);
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

    @Override
    public void periodic() {
        visibleTagPoses.clear();
        visibleTagIds.clear();
        estimatedPose = Optional.empty();

        PhotonPipelineResult latest = getLatestResult();

        if (latest != null) {
            for (PhotonTrackedTarget target : latest.getTargets()) {
                VisionConstants.kTagLayout.getTagPose(target.getFiducialId()).ifPresent(visibleTagPoses::add);
                visibleTagIds.add(target.getFiducialId());
            }

            DogLog.log("Vision/VisibleTagPoses", visibleTagPoses.toArray(new Pose3d[0]));

            estimatedPose = estimatePose(latest);
        }

        DogLog.log("Vision/SeeingAprilTag", estimatedPose.isPresent());

        if (estimatedPose.isPresent()) {
            visionField.setRobotPose(getEstimatedPose2d().get());

            if (DriverStation.isTeleop()) {
                adjustDrivetrainPose();

                if (!hasSeededPose) {
                    hasSeededPose = true;
                    drivetrain.resetPose(getEstimatedPose2d().get());
                }
            }
        }

        estimatedPose.ifPresent(pose -> DogLog.log("Vision/EstimatedPose", pose.estimatedPose));
    }

    /**
     * Returns the newest unread pipeline result if it has targets and isn't too
     * stale to trust, or {@code null} otherwise.
     */
    private PhotonPipelineResult getLatestResult() {
        List<PhotonPipelineResult> results = photonCam.getAllUnreadResults();
        if (results.isEmpty()) {
            return null;
        }

        PhotonPipelineResult latest = results.get(results.size() - 1);
        boolean isStale = Timer.getTimestamp() - latest.getTimestampSeconds() > VisionConstants.kMaxResultAgeSeconds;

        return (latest.hasTargets() && !isStale) ? latest : null;
    }

    /**
     * Estimates a robot pose from a fresh, non-empty pipeline result, if the
     * target(s) are trustworthy enough.
     */
    private Optional<EstimatedRobotPose> estimatePose(PhotonPipelineResult result) {
        List<PhotonTrackedTarget> targets = result.getTargets();

        if (targets.size() > 1) {
            multiTag = true;
            return visionPoseEstimator.estimateCoprocMultiTagPose(result);
        }

        PhotonTrackedTarget bestTarget = result.getBestTarget();
        if (bestTarget.poseAmbiguity < VisionConstants.kMaxSingleTagAmbiguity) {
            multiTag = false;
            return visionPoseEstimator.estimateLowestAmbiguityPose(result);
        }

        return Optional.empty();
    }

    public Optional<Pose2d> getEstimatedPose2d() {
        return estimatedPose.map(pose -> pose.estimatedPose.toPose2d());
    }

    public void adjustDrivetrainPose() {
        if (estimatedPose.isEmpty()) {
            return;
        }

        Translation2d visionTranslation = getEstimatedPose2d().get().getTranslation();
        Rotation2d driveTrainRotation = drivetrain.getPose().getRotation();
        Pose2d pose = new Pose2d(visionTranslation, driveTrainRotation);

        Matrix<N3, N1> stdDevs = multiTag ? VisionConstants.kMultiTagStdDevs : VisionConstants.kSingleTagStdDevs;

        drivetrain.addVisionMeasurement(pose, estimatedPose.get().timestampSeconds, stdDevs);
    }

    public void reseedPose() {
        hasSeededPose = false;
    }
}
