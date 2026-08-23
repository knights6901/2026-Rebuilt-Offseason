package frc.robot.subsystems.kicker;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem controlling the kicker wheel, which provides the final push
 * to transfer game pieces from the indexer into the shooter flywheel.
 * Uses a single TalonFX motor with closed-loop velocity control.
 */
public class Kicker extends SubsystemBase {
    private final TalonFX m_motorKicker = new TalonFX(KickerConstants.MotorId, new CANBus("rio"));

    /**
     * Initializes the kicker subsystem with motor configuration, PID settings,
     * and current limiting.
     */
    public Kicker() {
        m_motorKicker.getConfigurator().apply(KickerConstants.MotorConfig);
    }

    /**
     * Spins the kicker wheel at the configured velocity to feed a game piece into
     * the shooter.
     */
    public void kick() {
        m_motorKicker.setControl(new DutyCycleOut(KickerConstants.KickerPower));
    }

    public void kickReversed() {
        m_motorKicker.setControl(new DutyCycleOut(-0.85));
    }

    /** Stops the kicker motor by applying neutral output. */
    public void stop() {
        m_motorKicker.setControl(new NeutralOut());
    }
}