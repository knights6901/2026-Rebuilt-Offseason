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

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
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
    private VisionSim visionSim;

    private final PhotonPoseEstimator visionPoseEstimator;

    // not final because setting fieldLayout wasn't working without try/catch

    
    // for sim + logging so we can test easier
    private List<Pose3d> visibleTagPoses = new ArrayList<>();
    private List<Integer> visibleTagIds = new ArrayList<>();
    private final StructArrayPublisher<Pose3d> tagPublisher;

    /** The most recently estimated robot pose from vision (optional). */
    private Optional<EstimatedRobotPose> estimatedPose = Optional.empty();

    public boolean hasSeededPose = true;
    private boolean multiTag = false;

    public final Field2d m_visionfield = new Field2d();

    private final BooleanPublisher seeingAprilTagPub = NetworkTableInstance.getDefault()
            .getTable("Vision")
            .getBooleanTopic("April Tag?")
            .publish();

    /**
     * Creates the vision subsystem, initializing PhotonVision cameras and the
     * pose estimator from the current year's field layout. Starts camera
     * simulation if running in sim.
     */
    public Vision(Drive drivetrain) {
        photonCam = new PhotonCamera(VisionConstants.arducamName);

        if (Robot.isSimulation()) {
            visionSim = new VisionSim(drivetrain, photonCam);
        } else {
            visionSim = null;
        }

        this.drivetrain = drivetrain;

        if (VisionConstants.kTagLayout != null) {
            visionPoseEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout, VisionConstants.kRobotToCam);
        } else {
            visionPoseEstimator = null;
        }

        tagPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("visibleTags", Pose3d.struct).publish();

        SmartDashboard.putData("VisionField", m_visionfield);
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
        // clear the visible tag lists - they will be repopulated if there are targets
        // in the latest result
        visibleTagPoses.clear();
        visibleTagIds.clear();

        seeingAprilTagPub.set(estimatedPose.isPresent());

        List<PhotonPipelineResult> results = photonCam.getAllUnreadResults();

        if (results.size() == 0) {
            estimatedPose = Optional.empty();
            return;
        }

        PhotonPipelineResult latest = results.get(results.size() - 1);

        if (!latest.hasTargets() || Timer.getTimestamp() - latest.getTimestampSeconds() > 0.6901) {
            estimatedPose = Optional.empty();
            return;
        }

        List<PhotonTrackedTarget> targets = latest.getTargets();
        PhotonTrackedTarget bestTarget = latest.getBestTarget();

        if (targets.size() > 1) {
            estimatedPose = visionPoseEstimator.estimateCoprocMultiTagPose(latest);
            multiTag = true;
        } else if (bestTarget.poseAmbiguity < 0.1) {
            estimatedPose = visionPoseEstimator.estimateLowestAmbiguityPose(latest);
            multiTag = false;
        } else {
            estimatedPose = Optional.empty();
            return;
        }

        for (PhotonTrackedTarget target : targets) {
            Optional<Pose3d> tagPose = VisionConstants.kTagLayout.getTagPose(target.getFiducialId());
            tagPose.ifPresent(visibleTagPoses::add);
            visibleTagIds.add(target.getFiducialId());
        }

        tagPublisher.set(visibleTagPoses.toArray(new Pose3d[0]));

        if (estimatedPose.isPresent()) {
            m_visionfield.setRobotPose(getEstimatedPose2d().get());

            if (DriverStation.isTeleop()) {
                // adjustDrivetrainPose();

                if (!hasSeededPose) {
                    hasSeededPose = true;
                    drivetrain.resetPose(getEstimatedPose2d().get());
                }
            }
        }
    }

    public Optional<Pose2d> getEstimatedPose2d() {
        return estimatedPose.map(pose -> pose.estimatedPose.toPose2d());
    }

    public void adjustDrivetrainPose() {
        if (estimatedPose.isPresent()) {
            Translation2d visionPose = getEstimatedPose2d().get().getTranslation();
            Rotation2d driveTrainRotation = drivetrain.getPose().getRotation();
            Pose2d pose = new Pose2d(visionPose, driveTrainRotation);

            Matrix<N3, N1> stdDevs = multiTag ? VisionConstants.kMultiTagStdDevs : VisionConstants.kSingleTagStdDevs;

            drivetrain.addVisionMeasurement(pose, estimatedPose.get().timestampSeconds,
                    stdDevs);
        }
    }

    public void reseedPose() {
        hasSeededPose = false;
    }
}