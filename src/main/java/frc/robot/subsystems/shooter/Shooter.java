package frc.robot.subsystems.shooter;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.commands.ShootCommand;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.GameConstants;

/**
 * Subsystem controlling the dual-motor shooter flywheel.
 *
 * <p>
 * The left motor follows the right motor in the opposed direction. Provides
 * multiple shoot overloads for manual, automatic, and auto-aim velocity
 * control, as well as trajectory visualization and ballistics calculations
 * for distance-based shot speed.
 */
public class Shooter extends SubsystemBase {
    private final TalonFX m_motorRight = new TalonFX(ShooterConstants.RightMotorId, new CANBus("rio"));
    private final TalonFX m_motorLeft = new TalonFX(ShooterConstants.LeftMotorId, new CANBus("rio"));
    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    /** The possible states of the shooter mechanism. */
    public static enum ShooterState {
        OFF,
        AUTOHUB,
        AUTOHUB_PRIMING,
        AUTOPASS,
        AUTOPASS_PRIMING,
        PRIMING,
        MANUAL
    }

    private final StringPublisher shooterStatePub = NetworkTableInstance.getDefault()
            .getTable("Shooter")
            .getStringTopic("Shooter?")
            .publish();

    private final DoublePublisher targetRPSPub = NetworkTableInstance.getDefault()
            .getTable("Shooter")
            .getDoubleTopic("TargetRPS")
            .publish();

    private final DoublePublisher actualRPSPub = NetworkTableInstance.getDefault()
            .getTable("Shooter")
            .getDoubleTopic("ActualRPS")
            .publish();

    private final DoublePublisher manualRPSPub = NetworkTableInstance.getDefault()
            .getTable("Shooter")
            .getDoubleTopic("NaveenRPS")
            .publish();

    public ShooterState shooterState = ShooterState.OFF;

    // private AngularVelocity shootRPS = ShooterConstants.DefaultRPS;
    private AngularVelocity shootRPS = calculateRPS(Meters.of(3));
    public AngularVelocity targetRPS;

    public final Trigger primed = new Trigger(() -> {
        if (targetRPS == null) {
            return false;
        }

        double rpsError = getCurrentRPS().minus(targetRPS).abs(RotationsPerSecond);

        return rpsError <= ShooterConstants.PrimingTolerance.in(RotationsPerSecond);
    });

    /**
     * Configures both shooter motors with PID gains from constants and sets the
     * left motor to follow the right motor in the opposed direction.
     */
    public Shooter() {
        TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
        m_motorConfig.Slot0 = ShooterConstants.Gains;
        m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        m_motorRight.getConfigurator().apply(m_motorConfig);
        m_motorLeft.getConfigurator().apply(m_motorConfig);

        m_motorLeft.setControl(new Follower(ShooterConstants.RightMotorId, MotorAlignmentValue.Opposed));
    }

    public void shoot() {
        targetRPS = shootRPS;
        m_motorRight.setControl(m_request.withVelocity(shootRPS));
    }

    /**
     * Spins the flywheel at the specified angular velocity, typically used
     * for auto-aim shots where the velocity is computed from target distance.
     *
     * @param rps target angular velocity
     */
    public void shoot(AngularVelocity rps) {
        targetRPS = rps;
        m_motorRight.setControl(m_request.withVelocity(rps));
    }

    /** Stops the flywheel by applying neutral output to both motors. */
    public void stop() {
        m_motorRight.setControl(new NeutralOut());
        shooterState = ShooterState.OFF;
        targetRPS = RotationsPerSecond.of(0);
    }

    public void increaseShootRPS() {
        shootRPS = shootRPS.plus(RotationsPerSecond.of(1));
    }

    public void decreaseShootRPS() {
        shootRPS = shootRPS.minus(RotationsPerSecond.of(1));
    }

    /**
     * Calculates the required flywheel angular velocity to hit the hub target at
     * the given horizontal distance, using projectile kinematics with a fixed
     * launch pitch angle.
     *
     * @param groundDistance horizontal distance from the robot to the target
     * @param targetHeight   vertical height of the target from the floor
     * @return the flywheel angular velocity needed to reach the target
     */
    public AngularVelocity calculateRPS(Distance groundDistance, Distance targetHeight) {
        double dx = groundDistance.in(Meters);
        double dy = targetHeight.minus(ShooterConstants.BallExtakeHeight).in(Meters);

        double gVal = ShooterConstants.G.in(MetersPerSecondPerSecond);
        double pitchRad = ShooterConstants.Pitch.in(Radians);

        double numerator = gVal * dx * dx;
        double denominator = 2 * Math.pow(Math.cos(pitchRad), 2) * (dx * Math.tan(pitchRad) - dy);

        double velocity = Math.sqrt(numerator / denominator);

        double rps = velocity / (2 * Math.PI * 0.051);

        double damping = groundDistance.lte(ShooterConstants.NearHubDistance)
                ? ShooterConstants.DampingNearCoefficient
                : ShooterConstants.DampingFarCoefficient;

        return RotationsPerSecond.of(damping * rps);
    }

    /**
     * Calculates the required flywheel angular velocity to hit the hub target at
     * the given horizontal distance, using projectile kinematics with a fixed
     * launch pitch angle.
     *
     * @param groundDistance horizontal distance from the robot to the target
     * @return the flywheel angular velocity needed to reach the target
     */
    public AngularVelocity calculateRPS(Distance groundDistance) {
        return calculateRPS(groundDistance, GameConstants.HubTargetHeight);
    }

    public AngularVelocity getShootRPS() {
        return shootRPS;
    }

    public AngularVelocity getAPManualRPS(double axisInput) {
        return ShooterConstants.MaxRPS.times(axisInput);
    }

    public AngularVelocity getCurrentRPS() {
        return m_motorLeft.getVelocity().getValue();
    }

    public AngularVelocity getTargetRPS() {
        return targetRPS;
    }

    @Override
    public void periodic() {
        // Publish current velocities for telemetry
        if (shooterState == ShooterState.MANUAL) {
            shooterStatePub.set(shootRPS.in(RotationsPerSecond) + " (MANUAL)");
        } else if (shooterState == ShooterState.AUTOHUB_PRIMING) {
            shooterStatePub.set("PRIMING AUTOHUB");
        } else if (shooterState == ShooterState.AUTOPASS_PRIMING) {
            shooterStatePub.set("PRIMING AUTOPASS");
        } else {
            shooterStatePub.set(shooterState.toString());
        }

        actualRPSPub.set(getCurrentRPS().in(RotationsPerSecond));

        targetRPSPub.set(targetRPS != null ? targetRPS.in(RotationsPerSecond) : 0);

        manualRPSPub.set(shootRPS.in(RotationsPerSecond));

        // nearFarPub.set();
    }

    /**
     * Creates a command that primes the shooter to the default prime RPS.
     */
    public Command prime() {
        return run(() -> {
            shoot(ShooterConstants.DefaultPrimeRPS);
            shooterState = ShooterState.PRIMING;
        });
    }

    public Command manuallyShoot(
            Supplier<AngularVelocity> rpsSupplier,
            Kicker kicker,
            Indexer indexer) {
        return new ShootCommand(
                this, kicker, indexer,
                rpsSupplier,
                ShooterState.PRIMING,
                ShooterState.MANUAL);
    }

    /**
     * Creates a command that automatically calculates the required shooter RPS
     * based on the robot's current position and a specified target location.
     */
    private Command shootAtTarget(
            Supplier<Pose2d> currentPoseSupplier,
            Translation3d target,
            Kicker kicker,
            Indexer indexer,
            ShooterState primingState,
            ShooterState shootingState) {
        Supplier<AngularVelocity> rpsSupplier = () -> {
            Pose2d currentPose = currentPoseSupplier.get();
            Translation2d xyProjection = new Translation2d(
                    target.getMeasureX(), target.getMeasureY());

            Distance shotGroundDistance = Meters
                    .of(currentPose.getTranslation().getDistance(xyProjection));

            return calculateRPS(shotGroundDistance, target.getMeasureZ());
        };

        return new ShootCommand(
                this, kicker, indexer,
                rpsSupplier,
                primingState, shootingState);
    }

    public Command autoAimShoot(
            Supplier<Pose2d> currentPoseSupplier,
            Kicker kicker,
            Indexer indexer) {
        Translation2d hubXY = GameConstants.getHubLocation();
        Translation3d hub = new Translation3d(
                hubXY.getMeasureX(),
                hubXY.getMeasureY(),
                GameConstants.HubTargetHeight);

        return shootAtTarget(
                currentPoseSupplier,
                hub,
                kicker, indexer,
                ShooterState.AUTOHUB_PRIMING, ShooterState.AUTOHUB);
    }

    public Command passShoot(
            Supplier<Pose2d> currentPoseSupplier,
            Kicker kicker,
            Indexer indexer) {
        return shootAtTarget(
                currentPoseSupplier,
                new Translation3d(GameConstants.getPassLocation(currentPoseSupplier.get())),
                kicker, indexer,
                ShooterState.AUTOPASS_PRIMING, ShooterState.AUTOPASS);
    }
}
