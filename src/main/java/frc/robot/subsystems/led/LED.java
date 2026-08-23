package frc.robot.subsystems.led;
import frc.robot.subsystems.drive.Drive;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;
    private final AddressableLEDBufferView left;
    private final AddressableLEDBufferView middle;
    private final AddressableLEDBufferView right;

    public LED(Drive drivetrain) {
        led = new AddressableLED(LEDConstants.Port);
        buffer = new AddressableLEDBuffer(LEDConstants.Length);
        led.setLength(LEDConstants.Length);

        left = buffer.createView(0, 36);
        middle = buffer.createView(37, 166);
        right = buffer.createView(167, 185);

        led.start();
    }

    public LEDPattern fireUpPattern(double t) {
        t = Math.max(0.0, Math.min(1.0, t)); // clamp

        if (t < 0.33) {
            // red → orange
            double local = t / 0.33;
            return LEDPattern.solid(new Color(1.0, 0.27 * local, 0.0));
        } else if (t < 0.66) {
            // orange → yellow
            double local = (t - 0.33) / 0.33;
            return LEDPattern.solid(new Color(1.0, 0.27 + 0.73 * local, 0.0));
        } else {
            // yellow → green
            double local = (t - 0.66) / 0.34;
            return LEDPattern.solid(new Color(1.0 - local, 1.0, 0.0));
        }
    }

    /**
     * Creates a command that runs a pattern on the entire LED strip.
     *
     * @param pattern the LED pattern to run
     */
    public Command runPattern(LEDPattern pattern) {
        return run(() -> pattern.applyTo(buffer));
    }

    public Command runPatternLeft(LEDPattern pattern) {
        return new RunCommand(() -> pattern.applyTo(left));
    }

    public Command runPatternMiddle(LEDPattern pattern) {
        return new RunCommand(() -> pattern.applyTo(middle));
    }

    public Command runPatternRight(LEDPattern pattern) {
        return new RunCommand(() -> pattern.applyTo(right));
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