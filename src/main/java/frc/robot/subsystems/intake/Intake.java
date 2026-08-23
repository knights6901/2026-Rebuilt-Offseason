package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.intake.IntakeConstants.*;
import frc.robot.Constants.CANConstants;

/**
 * Controls the intake rollers via a single TalonFX with closed-loop velocity
 * control.
 */
public class Intake extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(MotorId, CANConstants.kSubsystemNetwork);

    private boolean intaking = false;

    public static enum IntakeState {
        OFF,
        INTAKING,
        REVERSED
    }

    private IntakeState intakeState = IntakeState.OFF;

    public Intake() {
        m_motor.getConfigurator().apply(MotorConfig);
    }

    /** Returns a command that runs the rollers inward at the default velocity. */
    public Command intake() {
        return intake(IntakeRPS);
    }

    /** Returns a command that runs the rollers inward at {@code rps}. */
    public Command intake(AngularVelocity rps) {
        return run(() -> {
            m_motor.setControl(new VelocityVoltage(rps));
            intaking = true;
        });
    }

    /** Returns a command that ejects game pieces at the default velocity. */
    public Command outtake() {
        return outtake(IntakeRPS.times(.2));
    }

    /**
     * Returns a command that ejects game pieces at {@code rps} (negated
     * internally).
     */
    public Command outtake(AngularVelocity rps) {
        return run(() -> {
            m_motor.setControl(new VelocityVoltage(rps.times(-1.0)));
            intaking = false;
        });
    }

    /** Stops the intake motor by applying neutral output. */
    public Command stop() {
        return run(() -> {
            m_motor.setControl(new NeutralOut());
            intaking = false;
        });
    }

    /** Whether the intake is actively intaking. */
    public boolean currentlyIntaking() {
        return intaking;
    }

    @Override
    public void periodic() {
        if (intaking && m_motor.getVelocity().getValueAsDouble() < -1) {
            intakeState = IntakeState.REVERSED;
        } else if (intaking) {
            intakeState = IntakeState.INTAKING;
        } else {
            intakeState = IntakeState.OFF;
        }

        DogLog.log("Intake/State", intakeState.toString());
    }
}
