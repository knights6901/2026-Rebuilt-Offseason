package frc.robot.subsystems.kicker;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    /** Stops the kicker motor by applying neutral output. */
    public Command stop() {
        return run(() -> m_motor.setControl(new NeutralOut()));
    }

    @Override
    public void periodic() {
        DogLog.log("Kicker/CurrentRPS", m_motor.getVelocity().getValue());
    }
}
