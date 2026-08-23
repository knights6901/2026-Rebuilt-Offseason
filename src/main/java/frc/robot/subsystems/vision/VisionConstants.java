package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import edu.wpi.first.math.geometry.Transform3d;
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
                new Rotation3d(0, Math.PI / 6, Math.PI));

        /** The layout of AprilTags on the field for localization. */
        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout
                        .loadField(AprilTagFields.kDefaultField);

        // values need to be heavily tested and tuned
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
}
