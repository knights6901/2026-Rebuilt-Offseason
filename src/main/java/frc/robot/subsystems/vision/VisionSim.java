package frc.robot.subsystems.vision;

import frc.robot.subsystems.drive.Drive;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.vision.VisionConstants.*;

/**
 * Simulates a PhotonVision camera seeing AprilTags. {@link Vision} only
 * constructs this subsystem when running in simulation, so no sim/real
 * branching is needed here.
 *
 * <p>
 * The camera is deliberately <b>not</b> rendered from
 * {@link Drive#getPose()}. That pose is corrected by vision, so using it would
 * close a feedback loop: the camera would render from the pose vision is
 * itself moving, any bias would never be corrected away, and the pose would
 * walk off in the direction of the bias while the measured residual stayed
 * near zero. Instead we keep a private odometry-only pose, fed from the raw
 * gyro heading and module positions, which vision can never touch. In sim this
 * is ground truth, since the module simulation models no wheel slip.
 */
public class VisionSim extends SubsystemBase {
    private final VisionSystemSim visionSim;
    private final Drive drivetrain;

    /**
     * Vision-free pose used as ground truth for rendering. Built lazily because
     * module positions aren't populated until the odometry thread's first
     * successful acquisition.
     */
    private SwerveDriveOdometry groundTruthOdometry;

    public VisionSim(Drive drivetrain, PhotonCamera photonCam) {
        this.drivetrain = drivetrain;

        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(kTagLayout);

        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(kSimCameraWidthPx, kSimCameraHeightPx, kSimCameraFov);
        cameraProp.setCalibError(kSimCalibErrorAvgPx, kSimCalibErrorStdDevPx);
        cameraProp.setFPS(kSimFps);
        cameraProp.setAvgLatencyMs(kSimAvgLatencyMs);
        cameraProp.setLatencyStdDevMs(kSimLatencyStdDevMs);

        PhotonCameraSim cameraSim = new PhotonCameraSim(photonCam, cameraProp);
        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);
        cameraSim.enableDrawWireframe(true);

        visionSim.addCamera(cameraSim, kRobotToCam);

        SmartDashboard.putData("VisionSim", visionSim.getDebugField());
    }

    /**
     * Teleports the simulated robot's ground-truth pose. Call this alongside
     * {@link Drive#resetPose} in sim whenever the robot is deliberately moved (an
     * auto starting position, for instance), so ground truth and the estimate stay
     * consistent.
     */
    public void resetGroundTruth(Pose2d pose) {
        var state = drivetrain.getState();
        if (state.ModulePositions == null) {
            return;
        }

        if (groundTruthOdometry == null) {
            groundTruthOdometry = new SwerveDriveOdometry(
                    drivetrain.getKinematics(), state.RawHeading, state.ModulePositions, pose);
            return;
        }

        groundTruthOdometry.resetPosition(state.RawHeading, state.ModulePositions, pose);
    }

    @Override
    public void simulationPeriodic() {
        var state = drivetrain.getState();
        if (state.ModulePositions == null) {
            return;
        }

        if (groundTruthOdometry == null) {
            groundTruthOdometry = new SwerveDriveOdometry(
                    drivetrain.getKinematics(), state.RawHeading, state.ModulePositions, drivetrain.getPose());
        }

        Pose2d groundTruth = groundTruthOdometry.update(state.RawHeading, state.ModulePositions);

        DogLog.log("VisionSim/GroundTruthPose", groundTruth);
        DogLog.log("VisionSim/EstimateError",
                drivetrain.getPose().getTranslation().minus(groundTruth.getTranslation()).getNorm());

        visionSim.update(groundTruth);
    }
}
