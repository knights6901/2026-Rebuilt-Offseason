package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.GameConstants;

/**
 * Projectile-motion math converting a shot distance into the required flywheel
 * speed.
 */
public final class ShooterPhysics {
    private ShooterPhysics() {
    }

    /**
     * Calculates the flywheel velocity to hit a target at the given ground
     * distance and height, using projectile kinematics at a fixed launch pitch.
     */
    public static AngularVelocity calculateRPS(Distance groundDistance, Distance targetHeight) {
        double dx = groundDistance.in(Meters);
        double dy = targetHeight.minus(BallExtakeHeight).in(Meters);

        double gVal = G.in(MetersPerSecondPerSecond);
        double pitchRad = Pitch.in(Radians);

        double numerator = gVal * dx * dx;
        double denominator = 2 * Math.pow(Math.cos(pitchRad), 2) * (dx * Math.tan(pitchRad) - dy);

        double exitVelocity = Math.sqrt(numerator / denominator);

        double rps = exitVelocity / (2 * Math.PI * WheelRadius.in(Meters));

        double damping = groundDistance.lte(NearHubDistance)
                ? DampingNearCoefficient
                : DampingFarCoefficient;

        return RotationsPerSecond.of(damping * rps);
    }

    /**
     * Calculates the flywheel velocity to hit the hub at {@code groundDistance}.
     */
    public static AngularVelocity calculateRPS(Distance groundDistance) {
        return calculateRPS(groundDistance, GameConstants.HubTargetHeight);
    }
}
