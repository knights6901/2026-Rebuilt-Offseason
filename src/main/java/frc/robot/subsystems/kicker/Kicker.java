package frc.robot.subsystems.kicker;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.kicker.KickerConstants.*;
import frc.robot.Constants.CANConstants;

/**
 * Controls the kicker wheel that pushes game pieces from the indexer into the
 * shooter.
 */
public class Kicker extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(MotorId, CANConstants.kSubsystemNetwork);

    public Kicker() {
        m_motor.getConfigurator().apply(MotorConfig);
    }

    /** Returns a command that spins the kicker wheel at the configured velocity. */
    public Command kick() {
        return run(() -> m_motor.setControl(new VelocityVoltage(KickerPower)));
    }

    /** Returns a command that spins the kicker wheel in reverse. */
    public Command kickReversed() {
        return run(() -> m_motor.setControl(new VelocityVoltage(KickerPower.times(-1))));
    }

    /**
     * Returns a command that kicks like {@link #kick()}, but briefly reverses the
     * wheel whenever it has been jammed for {@code JamTime}, then resumes kicking.
     */
    public Command kickWithUnjam() {
        Debouncer jammed = new Debouncer(JamTime.in(Seconds));

        return kick()
                .beforeStarting(() -> jammed.calculate(false))
                .until(() -> jammed.calculate(isStalled()))
                .andThen(kickReversed().withTimeout(UnjamTime))
                .repeatedly();
    }

    /** Whether the wheel is drawing high current while well below kicking speed. */
    private boolean isStalled() {
        return Math.abs(m_motor.getStatorCurrent().getValueAsDouble()) > JamCurrent.in(Amps)
                && m_motor.getVelocity().getValue().lt(JamVelocity);
    }

    /** Stops the kicker motor by applying neutral output. */
    public Command stop() {
        return run(() -> m_motor.setControl(new NeutralOut()));
    }
}
