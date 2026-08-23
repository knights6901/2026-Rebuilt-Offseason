package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem controlling the indexer mechanism, which transfers game pieces
 * from the intake into the shooter using a single TalonFX motor with
 * closed-loop velocity control.
 */
public class Indexer extends SubsystemBase {
    private final TalonFX m_motorIndexer = new TalonFX(IndexerConstants.MotorId, new CANBus("rio"));

    /**
     * Initializes the indexer subsystem with motor configuration and PID settings.
     */
    public Indexer() {
        m_motorIndexer.getConfigurator().apply(IndexerConstants.MotorConfig);
    }

    /**
     * Runs the indexer motor at the configured velocity to feed game pieces
     * toward the shooter.
     */
    public void enable() {
        // m_motorIndexer.setControl(m_request.withVelocity(Power));
        m_motorIndexer.setControl(new DutyCycleOut(0.85));
    }

    /**
     * Runs the indexer motor in the reverse direction at the configured velocity.
     */
    public void enableInverted() {
        // m_motorIndexer.setControl(new VelocityVoltage(Power.times(-1)));
        // m_motorIndexer.setControl(m_request.withVelocity(Power.times(-1)));
        m_motorIndexer.setControl(new DutyCycleOut(-0.25));
    }

    /** Stops the indexer motor by commanding zero velocity. */
    public void stop() {
        m_motorIndexer.setControl(new NeutralOut());
    }
}
