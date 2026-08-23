package frc.robot.subsystems.kicker;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.kicker.KickerConstants.*;
import frc.robot.Constants.CANConstants;

/**
 * Subsystem controlling the kicker wheel, which provides the final push
 * to transfer game pieces from the indexer into the shooter flywheel.
 */
public class Kicker extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(MotorId, CANConstants.kSubsystemNetwork);

    /**
     * Initializes the kicker subsystem with motor configuration, PID settings,
     * and current limiting.
     */
    public Kicker() {
        m_motor.getConfigurator().apply(MotorConfig);
    }

    /**
     * Returns a command that spins the kicker wheel
     * at the configured velocity to feed a game piece into the shooter.
     */
    public Command kick() {
        return run(() -> m_motor.setControl(new VelocityVoltage(KickerPower)));
    }

    /**
     * Returns a command that spins the kicker wheel in the reverse direction of
     * the configured velocity to feed a game piece into the shooter.
     */
    public Command kickReversed() {
        return run(() -> m_motor.setControl(new VelocityVoltage(KickerPower.times(-1))));
    }

    /** Stops the kicker motor by applying neutral output. */
    public Command stop() {
        return run(() -> m_motor.setControl(new NeutralOut()));
    }
}
