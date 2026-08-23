package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem controlling the intake rollers, used to pull game pieces into
 * the robot or eject them. Driven by a single TalonFX motor with closed-loop
 * velocity control.
 */
public class Intake extends SubsystemBase {
    private final TalonFX m_motorIntake = new TalonFX(IntakeConstants.MotorId, new CANBus("rio"));
    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    private boolean intaking = false;

    public static enum IntakeState {
        OFF,
        INTAKING,
        REVERSED
    }

    private IntakeState intakeState = IntakeState.OFF;

    private final StringPublisher intakeStatePub = NetworkTableInstance.getDefault()
            .getTable("Intake")
            .getStringTopic("Intake?")
            .publish();

    /**
     * Initializes the intake subsystem with motor configuration and PID settings.
     */
    public Intake() {
        m_motorIntake.getConfigurator().apply(IntakeConstants.MotorConfig);
    }

    /** Runs the intake rollers inward at the default velocity. */
    public void intake() {
        intake(IntakeConstants.IntakeRPS);
    }

    /**
     * Runs the intake rollers inward at a custom velocity.
     *
     * @param rps target angular velocity for the intake motor
     */
    public void intake(AngularVelocity rps) {
        m_motorIntake.setControl(m_request.withVelocity(rps));
        intaking = true;
    }

    /**
     * Runs the intake rollers outward at the default velocity to eject game pieces.
     */
    public void outtake() {
        outtake(IntakeConstants.IntakeRPS.times(.2));
    }

    /**
     * Runs the intake rollers outward at a custom velocity to eject game pieces.
     *
     * @param rps target angular velocity magnitude (will be negated internally)
     */
    public void outtake(AngularVelocity rps) {
        m_motorIntake.setControl(m_request.withVelocity(rps.times(-1.0)));
        intaking = false;
    }

    /** Stops the intake motor by applying neutral output. */
    public void stop() {
        m_motorIntake.setControl(new NeutralOut());
        intaking = false;
    }

    /**
     * Gets the current intake state.
     *
     * @return {@code true} if the intake is actively intaking, {@code false}
     *         otherwise
     */
    public boolean currentlyIntaking() {
        return intaking;
    }

    @Override
    public void periodic() {
        if (intaking && m_motorIntake.getVelocity().getValueAsDouble() < -1) {
            intakeState = IntakeState.REVERSED;
        } else if (intaking) {
            intakeState = IntakeState.INTAKING;
        } else {
            intakeState = IntakeState.OFF;
        }
        intakeStatePub.set(intakeState.toString());
    }
}

