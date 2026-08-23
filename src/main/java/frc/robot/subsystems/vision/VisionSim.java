package frc.robot.subsystems.vision;

import frc.robot.subsystems.drive.Drive;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Simulates a PhotonVision camera seeing AprilTags, driven by the drivetrain's
 * simulated pose. {@link Vision} only constructs this subsystem when running
 * in simulation, so no sim/real branching is needed here.
 */
public class VisionSim extends SubsystemBase {
    private final VisionSystemSim visionSim;
    private final Drive drivetrain;

    public VisionSim(Drive drivetrain, PhotonCamera photonCam) {
        this.drivetrain = drivetrain;

        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(VisionConstants.kTagLayout);

        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
        cameraProp.setCalibError(.25, 0.88);
        cameraProp.setFPS(60);
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        PhotonCameraSim cameraSim = new PhotonCameraSim(photonCam, cameraProp);
        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);
        cameraSim.enableDrawWireframe(true);

        visionSim.addCamera(cameraSim, VisionConstants.kRobotToCam);

        SmartDashboard.putData("VisionSim", visionSim.getDebugField());
    }

    @Override
    public void simulationPeriodic() {
        visionSim.update(drivetrain.getPose());
    }
}
