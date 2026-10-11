package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.GameConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

/**
 * Rotates in place until the robot is aimed at our alliance's hub, then ends.
 */
public class AlignToHub extends AlignShooter {
    public AlignToHub(Drive drivetrain, Vision vision) {
        super(drivetrain, () -> vision.getHubBearing().orElseGet(() -> getHubBearingFromPose(drivetrain)));
    }

    /** The robot-relative bearing to the hub according to the drivetrain pose. */
    private static Rotation2d getHubBearingFromPose(Drive drivetrain) {
        Pose2d pose = drivetrain.getPose();

        return GameConstants.getHubLocation()
                .minus(pose.getTranslation())
                .getAngle()
                .minus(pose.getRotation());
    }
}
