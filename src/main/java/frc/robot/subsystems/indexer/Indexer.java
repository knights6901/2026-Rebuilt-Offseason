package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.indexer.IndexerConstants.*;
import frc.robot.Constants.CANConstants;

/**
 * Controls the indexer that feeds game pieces from the intake into the shooter.
 */
public class Indexer extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(MotorId, CANConstants.kSubsystemNetwork);

    public Indexer() {
        m_motor.getConfigurator().apply(MotorConfig);
    }

    /** Returns a command that runs the indexer at the configured velocity. */
    public Command enable() {
        return run(() -> m_motor.setControl(new VelocityVoltage(Power)));
    }

    /** Returns a command that runs the indexer in reverse. */
    public Command enableInverted() {
        return run(() -> m_motor.setControl(new VelocityVoltage(Power.times(-1))));
    }

    /** Stops the indexer motor by applying neutral output. */
    public Command stop() {
        return run(() -> m_motor.setControl(new NeutralOut()));
    }

    @Override
    public void periodic() {
        DogLog.log("Indexer/CurrentRPS", m_motor.getVelocity().getValue());
    }
}
