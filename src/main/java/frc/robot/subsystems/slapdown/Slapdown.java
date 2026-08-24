package frc.robot.subsystems.slapdown;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
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

    /**
     * The period the simulation is advanced by on each call to
     * {@link #simulationPeriodic()}.
     */
    private static final double kSimLoopPeriod = 0.020;

    /**
     * Degrees of arm elevation gained per rotation of the motor. Negative: winding
     * the motor forward swings the arm down and out.
     */
    private static final double kDegreesPerMotorRotation = DeployedAngle.minus(HomeAngle).in(Degrees)
            / IntakePosition.minus(HomePosition).in(Rotations);

    private final TalonFX m_motor = new TalonFX(MotorId, CANConstants.kSubsystemNetwork);
    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    /**
     * Physics model of the arm, in the arm's own angular frame. Only ever advanced
     * in simulation.
     */
    private final SingleJointedArmSim m_armSim = new SingleJointedArmSim(
            Gearbox,
            EffectiveGearRatio,
            SingleJointedArmSim.estimateMOI(ArmLength.in(Meters), ArmMass.in(Kilograms)),
            ArmLength.in(Meters),
            Math.min(HomeAngle.in(Radians), DeployedAngle.in(Radians)),
            Math.max(HomeAngle.in(Radians), DeployedAngle.in(Radians)),
            SimulateGravity,
            HomeAngle.in(Radians));

    /**
     * True once the arm has settled at its target (not
     * {@link SlapdownState#MOVING}).
     */
    public final Trigger atTarget = new Trigger(() -> getDeploymentState() != SlapdownState.MOVING);

    /** Configures the motor, resets its position to home, and holds it there. */
    public Slapdown() {
        m_motor.getConfigurator().apply(MotorConfig);
        resetSlapdownPosition();

        // Latch a hold on the home position so the arm is held up the moment the
        // robot is enabled, rather than sagging under gravity until something
        // commands it. Phoenix keeps the request applied until it is replaced.
        m_motor.setControl(m_request.withPosition(HomePosition));
    }

    /** Returns a command that deploys the arm to the intake position. */
    public Command slapdown() {
        return run(() -> m_motor.setControl(m_request.withPosition(IntakePosition)))
                .until(() -> getDeploymentState() == SlapdownState.DOWN);
    }

    /** Returns a command that retracts the arm to the home position. */
    public Command retractSlapdown() {
        return run(() -> m_motor.setControl(m_request.withPosition(HomePosition)))
                .until(() -> getDeploymentState() == SlapdownState.UP);
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

    /**
     * The arm's elevation above horizontal, interpolated between
     * {@link SlapdownConstants#HomeAngle} and
     * {@link SlapdownConstants#DeployedAngle}. The two calibrated endpoints are
     * the only trustworthy relationship between motor rotations and arm travel,
     * since {@link SlapdownConstants#IntakePosition} also takes up the
     * mechanism's slack. Both the visualization and the sim go through this
     * mapping.
     */
    public Angle getArmAngle() {
        return armAngleOf(getSlapdownPosition());
    }

    /** The arm elevation corresponding to a motor position. */
    private static Angle armAngleOf(Angle motorPosition) {
        return HomeAngle.plus(Degrees.of(
                kDegreesPerMotorRotation * motorPosition.minus(HomePosition).in(Rotations)));
    }

    /**
     * The motor position corresponding to an arm elevation. The inverse of
     * {@link #armAngleOf}.
     */
    private static Angle motorPositionOf(Angle armAngle) {
        return HomePosition.plus(Rotations.of(
                armAngle.minus(HomeAngle).in(Degrees) / kDegreesPerMotorRotation));
    }

    /**
     * The pose of the arm relative to the robot origin, for AdvantageScope's
     * component visualization. The arm's model is zeroed at the robot origin, so
     * this both rotates it about its hinge and moves it out to
     * {@link SlapdownConstants#PivotOffset}.
     *
     * <p>
     * A positive pitch about {@code +Y} tips a forward-pointing arm downward, so
     * elevation above horizontal is the negative of the pitch.
     */
    public Pose3d getComponentPose() {
        double pitch = -getArmAngle().in(Radians) * DeployDirection;

        return new Pose3d(PivotOffset, new Rotation3d(0, pitch, 0));
    }

    @Override
    public void simulationPeriodic() {
        var simState = m_motor.getSimState();
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());

        m_armSim.setInputVoltage(simState.getMotorVoltage() * Math.signum(kDegreesPerMotorRotation));
        m_armSim.update(kSimLoopPeriod);

        simState.setRawRotorPosition(motorPositionOf(Radians.of(m_armSim.getAngleRads())));
        simState.setRotorVelocity(RotationsPerSecond.of(
                Radians.of(m_armSim.getVelocityRadPerSec()).in(Degrees) / kDegreesPerMotorRotation));
    }

    @Override
    public void periodic() {
        DogLog.log("Slapdown/State", getDeploymentState());
        DogLog.log("Slapdown/Position", getSlapdownPosition());

        DogLog.log("Slapdown/Angle", getArmAngle());

        DogLog.log("Slapdown/ComponentPoses", new Pose3d[] { getComponentPose() });
        DogLog.log("Slapdown/ZeroedPose", new Pose3d[] { new Pose3d() });
    }
}
