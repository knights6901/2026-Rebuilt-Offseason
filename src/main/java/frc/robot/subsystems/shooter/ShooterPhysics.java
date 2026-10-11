package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.GameConstants;

/**
 * Converts a shot distance into the flywheel speed needed to make it.
 *
 * <p>
 * The model has two halves. The flight half integrates a FUEL's path under
 * gravity, air drag and the Magnus force from its topspin, and searches for the exit
 * speed that brings the ball down through the target. The launch half turns
 * that exit speed into a drum speed: a ball rolling between the drum and the
 * fixed hood leaves at half the drum's surface speed, less losses, and the drum
 * slows as it hands momentum to the ball, so it has to start a little faster.
 *
 * <p>
 * None of that runs while shooting. The model is solved once, when the robot
 * code starts, into a table of drum speed against distance for each target
 * height, and a shot just interpolates within its table.
 */
public final class ShooterPhysics {
    /** Spacing of the distances the model is solved at to fill a table, in meters. */
    private static final double kTableSpacing = 0.1;

    /**
     * The furthest distance a table covers, in meters. Shots from beyond it get the
     * speed for this distance, which is already past what the shooter can reach.
     */
    private static final double kTableMaxDistance = 6.0;

    /** Time step of the flight integration, in seconds. */
    private static final double kTimeStep = 0.01;

    /** Longest flight the integration will follow, in seconds. */
    private static final double kMaxFlightTime = 6.0;

    /** Bisection steps in the exit speed search; each one halves the error. */
    private static final int kSolverIterations = 14;

    private static final double kGravity = G.in(MetersPerSecondPerSecond);
    private static final double kFuelRadius = FuelDiameter.in(Meters) / 2;
    private static final double kFuelMass = FuelMass.in(Kilograms);
    private static final double kWheelRadius = WheelRadius.in(Meters);

    /**
     * Drag and Magnus accelerations per unit of speed squared, in 1/m: the usual
     * {@code 1/2 * rho * C * A / m}.
     */
    private static final double kDragPerSpeedSquared = 0.5 * AirDensity * FuelDragCoefficient
            * Math.PI * kFuelRadius * kFuelRadius / kFuelMass;
    private static final double kMagnusPerSpeedSquared = 0.5 * AirDensity * FuelMagnusCoefficient
            * Math.PI * kFuelRadius * kFuelRadius / kFuelMass;

    /**
     * Drum speed, in rad/s, needed per m/s of exit speed. The first term is the
     * speed the drum must still have as the ball leaves it. The second is what the
     * drum loses during the shot, from conserving angular momentum between the drum
     * and a ball that comes out both moving and spinning.
     */
    private static final double kDrumSpeedPerExitSpeed = 2 / (ExitSpeedEfficiency * kWheelRadius)
            + kFuelMass * (1 + FuelInertiaFactor) * kWheelRadius / (2 * FlywheelInertia.in(KilogramSquareMeters));

    /** The exit speed the shooter produces flat out, in m/s. */
    private static final double kMaxExitSpeed = MaxRPS.in(RadiansPerSecond) / kDrumSpeedPerExitSpeed;

    /**
     * Drum speed, in rad/s, against ground distance, in meters, for each target
     * height, in meters, that has been asked for.
     */
    private static final Map<Double, InterpolatingDoubleTreeMap> kTables = new HashMap<>();

    static {
        /* Solve the two targets we shoot at up front, rather than mid-match. */
        tableFor(GameConstants.HubTargetHeight.in(Meters));
        tableFor(0);
    }

    private ShooterPhysics() {
    }

    /**
     * Calculates the flywheel velocity that lands a ball on a target at the given
     * ground distance and height. A target that is out of reach gets
     * {@link ShooterConstants#MaxRPS}.
     */
    public static AngularVelocity calculateRPS(Distance groundDistance, Distance targetHeight) {
        return RadiansPerSecond.of(tableFor(targetHeight.in(Meters)).get(groundDistance.in(Meters)));
    }

    /**
     * Calculates the flywheel velocity to hit the hub at {@code groundDistance}.
     */
    public static AngularVelocity calculateRPS(Distance groundDistance) {
        return calculateRPS(groundDistance, GameConstants.HubTargetHeight);
    }

    /**
     * Returns the table for a target at {@code targetHeight} meters off the floor,
     * solving the model to build it if this is the first time it's been needed.
     */
    private static InterpolatingDoubleTreeMap tableFor(double targetHeight) {
        return kTables.computeIfAbsent(targetHeight, height -> {
            InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();
            double heightGain = height - BallExtakeHeight.in(Meters);

            for (double distance = 0; distance <= kTableMaxDistance; distance += kTableSpacing) {
                table.put(distance, solveExitSpeed(distance, heightGain) * kDrumSpeedPerExitSpeed);
            }

            return table;
        });
    }

    /**
     * Finds the exit speed, in m/s, whose flight comes down through a point
     * {@code distance} away and {@code heightGain} above the shooter. Faster shots
     * always carry further, so this is a bisection between a standstill and the
     * fastest shot the shooter can make.
     */
    private static double solveExitSpeed(double distance, double heightGain) {
        double tooSlow = 0;
        double fastEnough = kMaxExitSpeed;

        for (int i = 0; i < kSolverIterations; i++) {
            double exitSpeed = (tooSlow + fastEnough) / 2;

            if (descentDistance(exitSpeed, heightGain) < distance) {
                tooSlow = exitSpeed;
            } else {
                fastEnough = exitSpeed;
            }
        }

        return fastEnough;
    }

    /**
     * Flies a ball launched at {@code exitSpeed} and returns how far away it is, in
     * meters, when it falls back through {@code heightGain} above the shooter.
     * Returns negative infinity if it never gets that high, so that such a shot
     * always counts as falling short.
     */
    private static double descentDistance(double exitSpeed, double heightGain) {
        double pitch = Pitch.in(Radians);

        double x = 0;
        double z = 0;
        double vx = exitSpeed * Math.cos(pitch);
        double vz = exitSpeed * Math.sin(pitch);

        for (double t = 0; t < kMaxFlightTime; t += kTimeStep) {
            /* Midpoint method: take a half step, then use the velocity and forces there. */
            double halfVx = vx + accelerationX(vx, vz) * kTimeStep / 2;
            double halfVz = vz + accelerationZ(vx, vz) * kTimeStep / 2;

            double nextX = x + halfVx * kTimeStep;
            double nextZ = z + halfVz * kTimeStep;
            vx += accelerationX(halfVx, halfVz) * kTimeStep;
            vz += accelerationZ(halfVx, halfVz) * kTimeStep;

            if (vz < 0 && nextZ < heightGain) {
                if (z < heightGain) {
                    return Double.NEGATIVE_INFINITY;
                }

                /* Crossed the target height during this step; find where. */
                double fraction = (z - heightGain) / (z - nextZ);
                return x + fraction * (nextX - x);
            }

            x = nextX;
            z = nextZ;
        }

        return x;
    }

    /**
     * Horizontal acceleration of a ball in flight: drag, plus the Magnus force.
     * Topspin pushes the ball at a right angle to its path, toward the side it
     * would curve down to, which is forward while it climbs and backward as it
     * falls.
     */
    private static double accelerationX(double vx, double vz) {
        double speed = Math.sqrt(vx * vx + vz * vz);

        return -kDragPerSpeedSquared * speed * vx + kMagnusPerSpeedSquared * speed * vz;
    }

    /** Vertical acceleration of a ball in flight: gravity, drag, and the Magnus force. */
    private static double accelerationZ(double vx, double vz) {
        double speed = Math.sqrt(vx * vx + vz * vz);

        return -kGravity - kDragPerSpeedSquared * speed * vz - kMagnusPerSpeedSquared * speed * vx;
    }
}
