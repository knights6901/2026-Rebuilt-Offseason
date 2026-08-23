package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

/**
 * Team-authored drivetrain constants.
 * <p>
 * Everything Tuner X owns (gains, CAN IDs, encoder offsets, gear ratios) lives
 * in {@link TunerConstants} and is overwritten on regeneration. Anything we tune
 * by hand belongs here.
 */
public final class DriveConstants {
    /** Fraction of the measured free speed we allow the driver to command. */
    private static final double kSpeedScalar = 1.0;

    /** Max commanded chassis speed, in meters per second. */
    public static final double kMaxSpeed = kSpeedScalar * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    /** Max commanded chassis rotation rate, in radians per second. */
    public static final double kMaxAngularRate = RotationsPerSecond.of(1.0).in(RadiansPerSecond);

    /** Fraction of full stick travel ignored around center, translation. */
    public static final double kTranslationDeadband = 0.1;

    /** Fraction of full stick travel ignored around center, rotation. */
    public static final double kRotationDeadband = 0.1;

    private DriveConstants() {
    }
}
