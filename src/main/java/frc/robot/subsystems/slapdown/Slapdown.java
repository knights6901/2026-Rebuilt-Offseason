package frc.robot.subsystems.slapdown;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.subsystems.slapdown.SlapdownConstants.*;
import frc.robot.Constants.CANConstants;

/**
 * Controls the slapdown mechanism, a hinged arm that deploys to a fixed
 * intake position and retracts to a home position. Driven by a single
 * TalonFX motor with closed-loop position control.
 */
public class Slapdown extends SubsystemBase {
    /** The operating states of the slapdown mechanism. */
    public static enum SlapdownState {
        UP,
        DOWN,
        MOVING
    }

    private final TalonFX m_motor = new TalonFX(MotorId, CANConstants.kSubsystemNetwork);
    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    /**
     * True once the arm has settled at its target (not
     * {@link SlapdownState#MOVING}).
     */
    public final Trigger atTarget = new Trigger(() -> getDeploymentState() != SlapdownState.MOVING);

    /** Configures the motor and resets its position to home. */
    public Slapdown() {
        m_motor.getConfigurator().apply(MotorConfig);
        resetSlapdownPosition();
    }

    /** Returns a command that deploys the arm to the intake position. */
    public Command slapdown() {
        return run(() -> m_motor.setControl(m_request.withPosition(IntakePosition))).until(atTarget);
    }

    /** Returns a command that retracts the arm to the home position. */
    public Command retractSlapdown() {
        return run(() -> m_motor.setControl(m_request.withPosition(HomePosition))).until(atTarget);
    }

    /**
     * Returns a command that drives the motor at {@code power} ([-1, 1]) for manual
     * control.
     */
    public Command setPower(double power) {
        return run(() -> m_motor.setControl(new DutyCycleOut(power)));
    }

    /** Stops the slapdown motor by applying neutral output. */
    public Command stop() {
        return run(() -> m_motor.setControl(new NeutralOut()));
    }

    /** Resets the motor's position encoder to zero (home position). */
    public void resetSlapdownPosition() {
        m_motor.setPosition(0);
    }

    /**
     * The current deployment state, derived from position error against each
     * target.
     */
    public SlapdownState getDeploymentState() {
        Angle position = getSlapdownPosition();

        if (position.minus(IntakePosition).abs(Degrees) < PositionTolerance.in(Degrees)) {
            return SlapdownState.DOWN;
        } else if (position.minus(HomePosition).abs(Degrees) < PositionTolerance.in(Degrees)) {
            return SlapdownState.UP;
        } else {
            return SlapdownState.MOVING;
        }
    }

    /** The current position of the slapdown motor. */
    public Angle getSlapdownPosition() {
        return m_motor.getPosition().getValue();
    }

    @Override
    public void periodic() {
        DogLog.log("Slapdown/State", getDeploymentState());
        DogLog.log("Slapdown/Position", getSlapdownPosition());
    }
}
