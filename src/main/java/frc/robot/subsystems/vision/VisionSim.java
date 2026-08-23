package frc.robot.subsystems.vision;
import frc.robot.subsystems.drive.Drive;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSim extends SubsystemBase {
    private VisionSystemSim visionSim;
    private PhotonCameraSim cameraSim;

    private Drive drivetrain;
    private PhotonCamera photonCam;
    
    public VisionSim(Drive drivetrain, PhotonCamera photonCam) {
        if (RobotBase.isSimulation()) {
            initializeSimulation(VisionConstants.kRobotToCam);
        } else {
            visionSim = null;
            cameraSim = null;
        }

        this.drivetrain = drivetrain;
        this.photonCam = photonCam;
    }

    /**
     * Sets up the PhotonVision simulation environment with a simulated camera
     * matching the physical camera's transform.
     *
     * @param robotToCam the 3D transform from the robot origin to the camera
     */
    private void initializeSimulation(Transform3d robotToCam) {
        visionSim = new VisionSystemSim("main");
        if (VisionConstants.kTagLayout != null) {
            visionSim.addAprilTags(VisionConstants.kTagLayout);
        }
        SimCameraProperties cameraProp = new SimCameraProperties();

        cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
        cameraProp.setCalibError(.25, 0.88);
        cameraProp.setFPS(60);
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);
        cameraSim = new PhotonCameraSim(photonCam, cameraProp);

        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);
        cameraSim.enableDrawWireframe(true);

        visionSim.addCamera(cameraSim, robotToCam);

        SmartDashboard.putData("VisionSim", visionSim.getDebugField());
    }

    @Override
    public void simulationPeriodic() {
        visionSim.update(drivetrain.getPose());
    }
}
