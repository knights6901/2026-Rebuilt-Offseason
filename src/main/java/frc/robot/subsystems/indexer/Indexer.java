package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.indexer.IndexerConstants.*;
import frc.robot.Constants.CANConstants;

/**
 * Subsystem controlling the indexer mechanism, which transfers game pieces
 * from the intake into the shooter using a single TalonFX motor with
 * closed-loop velocity control.
 */
public class Indexer extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(MotorId, CANConstants.kSubsystemNetwork);

    /**
     * Initializes the indexer subsystem with motor configuration and PID settings.
     */
    public Indexer() {
        m_motor.getConfigurator().apply(MotorConfig);
    }

    /**
     * Returns a command that runs the indexer motor at the configured velocity
     * to feed game pieces toward the shooter.
     */
    public Command enable() {
        return run(() -> m_motor.setControl(new VelocityVoltage(Power)));
    }

    /**
     * Returns a command that runs the indexer motor in the reverse direction of
     * the configured velocity.
     */
    public Command enableInverted() {
        return run(() -> m_motor.setControl(new VelocityVoltage(Power.times(-1))));
    }

    /** Stops the indexer motor by applying neutral output. */
    public Command stop() {
        return run(() -> m_motor.setControl(new NeutralOut()));
    }
}
