package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Constants that aren't owned by a single subsystem. Subsystem-specific values
 * belong in that subsystem's package.
 */
public final class Constants {
    public static final class Operator {
        public static final int kDriverControllerPort = 0;

        private Operator() {
        }
    }

    public static final class CANConstants {
        /// The CAN network for all non-sweve subsystems.
        public static final CANBus kSubsystemNetwork = new CANBus("rio");
    }

    public static final class GameConstants {
        /** The position of the hub/target on the blue alliance side of the field. */
        public static final Translation2d BlueHubLocation = new Translation2d(4.612, 4.021);
        /** The position of the hub/target on the red alliance side of the field. */
        public static final Translation2d RedHubLocation = new Translation2d(11.901, 4.021);

        /** The y-position of the left trench on the blue alliance side of the field. */
        public static final Distance BlueLeftTrenchY = Meters.of(7.435);
        /**
         * The y-position of the right trench on the blue alliance side of the field.
         */
        public static final Distance BlueRightTrenchY = Meters.of(0.634);

        /** The x-position of the pass-line on the blue alliance side of the field. */
        public static final Distance BluePassLineX = Meters.of(2.306);
        /** The x-position of the pass-line on the red alliance side of the field. */
        public static final Distance RedPassLineX = Meters.of(14.207);

        /** The pose of the left depot side on the blue alliance. */
        public static final Pose2d BlueLeftDepotPose = new Pose2d(
                new Translation2d(Inches.of(44.5), Inches.of(223.6)),
                new Rotation2d(Degrees.of(0)));

        /** The pose of the left depot side on the red alliance. */
        public static final Pose2d RedLeftDepotPose = new Pose2d(
                new Translation2d(Inches.of(606.72), Inches.of(94.09)),
                new Rotation2d(Degrees.of(180)));

        /** The height of the target hub from the ground. */
        public final static Distance HubTargetHeight = Meters.of(1.524);

        /** Returns the position of the hub based on the current alliance. */
        public static Translation2d getHubLocation() {
            return (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue) ? BlueHubLocation
                    : RedHubLocation;
        }

        /**
         * Returns the position of the pass line based on the current alliance and
         * drivetrain position.
         */
        public static Translation2d getPassLocation(Pose2d drivetrainPose) {
            Distance y = drivetrainPose.getMeasureY();

            return (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue)
                    ? new Translation2d(GameConstants.BluePassLineX, y)
                    : new Translation2d(GameConstants.RedPassLineX, y);
        }

        public static Pose2d getLeftDepotPose() {
            return (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue)
                    ? GameConstants.BlueLeftDepotPose
                    : GameConstants.RedLeftDepotPose;
        }
    }
}
