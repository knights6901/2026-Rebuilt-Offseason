package frc.robot.subsystems.slapdown;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Subsystem controlling the slapdown mechanism, a hinged arm that deploys
 * to a fixed intake position and retracts to a home position. Driven by a
 * single TalonFX motor with closed-loop position control.
 */
public class Slapdown extends SubsystemBase {
    /** The possible states of the slapdown mechanism. */
    public static enum SlapdownState {
        UP,
        DOWN,
        MOVING
    }

    private final TalonFX m_motorSlapdown = new TalonFX(SlapdownConstants.MotorId, new CANBus("rio"));
    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    public final Trigger atTarget = new Trigger(() -> getDeploymentState() != SlapdownState.MOVING);

    /**
     * 
     * Initializes the slapdown subsystem with motor configuration and PID settings.
     * Resets the motor position to home.
     */
    public Slapdown() {
        m_motorSlapdown.getConfigurator().apply(SlapdownConstants.MotorConfig);
        resetSlapdownPosition();
    }

    /** Moves the slapdown arm to the deployed intake position. */
    public void slapdown() {
        m_motorSlapdown.setControl(m_request.withPosition(SlapdownConstants.IntakePosition));
    }

    /** Retracts the slapdown arm to the stowed home position. */
    public void retractSlapdown() {
        m_motorSlapdown.setControl(m_request.withPosition(SlapdownConstants.HomePosition));
    }

    /**
     * Sets the slapdown motor to a specified duty cycle for manual control.
     *
     * @param power the duty cycle output from -1.0 to 1.0
     */
    public void setPower(double power) {
        m_motorSlapdown.setControl(new DutyCycleOut(power));
    }

    /** Stops the slapdown motor by applying neutral output. */
    public void stop() {
        m_motorSlapdown.setControl(new NeutralOut());
    }

    /**
     * Resets the slapdown motor position encoder to zero (home position).
     */
    public void resetSlapdownPosition() {
        m_motorSlapdown.setPosition(0);
    }

    /**
     * Returns whether the slapdown is currently moving, based on the error between
     * the current position and the target position for the current state.
     */
    public SlapdownState getDeploymentState() {
        Angle position = getSlapdownPosition();

        if (position.minus(SlapdownConstants.IntakePosition).abs(Degrees) < SlapdownConstants.PositionTolerance.in(Degrees)) {
            return SlapdownState.DOWN;
        } else if (position.minus(SlapdownConstants.HomePosition).abs(Degrees) < SlapdownConstants.PositionTolerance.in(Degrees)) {
            return SlapdownState.UP;
        } else {
            return SlapdownState.MOVING;
        }
    }

    /* Returns the current position of the slapdown motor. */
    public Angle getSlapdownPosition() {
        return m_motorSlapdown.getPosition().getValue();
    }

    /* Returns a command that deploys the slapdown when executed. */
    public Command slapdownCommand() {
        return run(() -> slapdown()).until(atTarget);
    }

    /* Returns a command that retracts the slapdown when executed. */
    public Command retractSlapdownCommand() {
        return run(() -> retractSlapdown()).until(atTarget);
    }
}
