package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.led.LEDConstants.*;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;

/** Drives the addressable LED strip, split into left/middle/right zones. */
public class LED extends SubsystemBase {
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;
    private final AddressableLEDBufferView left;
    private final AddressableLEDBufferView middle;
    private final AddressableLEDBufferView right;

    public LED(Drive drivetrain) {
        led = new AddressableLED(Port);
        buffer = new AddressableLEDBuffer(Length);
        led.setLength(Length);

        left = buffer.createView(0, 36);
        middle = buffer.createView(37, 166);
        right = buffer.createView(167, 185);

        led.start();
    }

    /** Returns a red→orange→yellow→green pattern interpolated by {@code t} in [0, 1]. */
    public LEDPattern fireUpPattern(double t) {
        t = Math.max(0.0, Math.min(1.0, t));

        if (t < 0.33) {
            double local = t / 0.33;
            return LEDPattern.solid(new Color(1.0, 0.27 * local, 0.0));
        } else if (t < 0.66) {
            double local = (t - 0.33) / 0.33;
            return LEDPattern.solid(new Color(1.0, 0.27 + 0.73 * local, 0.0));
        } else {
            double local = (t - 0.66) / 0.34;
            return LEDPattern.solid(new Color(1.0 - local, 1.0, 0.0));
        }
    }

    /** Returns a command that runs {@code pattern} on the entire strip. */
    public Command runPattern(LEDPattern pattern) {
        return run(() -> pattern.applyTo(buffer));
    }

    /** Returns a command that runs {@code pattern} on the left zone. */
    public Command runPatternLeft(LEDPattern pattern) {
        return run(() -> pattern.applyTo(left));
    }

    /** Returns a command that runs {@code pattern} on the middle zone. */
    public Command runPatternMiddle(LEDPattern pattern) {
        return run(() -> pattern.applyTo(middle));
    }

    /** Returns a command that runs {@code pattern} on the right zone. */
    public Command runPatternRight(LEDPattern pattern) {
        return run(() -> pattern.applyTo(right));
    }

    public Command runAllPatterns(
            Supplier<LEDPattern> patternLeft,
            Supplier<LEDPattern> patternMiddle,
            Supplier<LEDPattern> patternRight) {
        return run(() -> {
            patternLeft.get().applyTo(left);
            patternMiddle.get().applyTo(middle);
            patternRight.get().applyTo(right);
        });
    }

    public LEDPattern shooterPattern(Supplier<AngularVelocity> current, AngularVelocity target) {
        return fireUpPattern(current.get().in(RotationsPerSecond) / target.in(RotationsPerSecond));
    }

    @Override
    public void periodic() {
        led.setData(buffer);
    }
}