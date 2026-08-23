package frc.robot.subsystems.shooter;

import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.commands.ShootCommand;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.GameConstants;

/**
 * Dual-motor shooter flywheel; the left motor follows the right in the
 * opposed direction. Provides manual, automatic, and auto-aim shoot
 * commands. Ballistics math lives in {@link ShooterPhysics}.
 */
public class Shooter extends SubsystemBase {
    private final TalonFX m_motorRight = new TalonFX(RightMotorId, CANConstants.kSubsystemNetwork);
    private final TalonFX m_motorLeft = new TalonFX(LeftMotorId, CANConstants.kSubsystemNetwork);
    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    /** The operating states of the shooter mechanism. */
    public static enum ShooterState {
        /** Flywheel stopped. */
        OFF,
        /** Spinning up for an auto-aimed hub shot. */
        AUTOHUB_PRIMING,
        /** At speed and shooting into the hub. */
        AUTOHUB,
        /** Spinning up for an auto-aimed pass. */
        AUTOPASS_PRIMING,
        /** At speed and shooting a pass. */
        AUTOPASS,
        /** Spinning up for a manual shot. */
        PRIMING,
        /** At speed and shooting under manual/driver control. */
        MANUAL
    }

    /** The shooter's current operating state; set by the shoot commands. */
    public ShooterState shooterState = ShooterState.OFF;

    private AngularVelocity shootRPS = ShooterPhysics.calculateRPS(Meters.of(3));

    /** The flywheel RPS currently being targeted; zero when idle. */
    public AngularVelocity targetRPS = RotationsPerSecond.of(0);

    /**
     * True once the flywheel is within {@link ShooterConstants#PrimingTolerance} of
     * {@link #targetRPS}.
     */
    public final Trigger primed = new Trigger(
            () -> getCurrentRPS().minus(targetRPS).abs(RotationsPerSecond) <= PrimingTolerance.in(RotationsPerSecond));

    /** Configures both motors and sets the left to follow the right, opposed. */
    public Shooter() {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.Slot0 = Gains;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        m_motorRight.getConfigurator().apply(motorConfig);
        m_motorLeft.getConfigurator().apply(motorConfig);

        m_motorLeft.setControl(new Follower(RightMotorId, MotorAlignmentValue.Opposed));
    }

    /**
     * Spins the flywheel at the current manual target speed
     * ({@link #getShootRPS()}).
     */
    public void shoot() {
        shoot(shootRPS);
    }

    /** Spins the flywheel at {@code rps}, e.g. for auto-aim shots. */
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

    /** Bumps the manual target speed up by 1 RPS. */
    public void increaseShootRPS() {
        shootRPS = shootRPS.plus(RotationsPerSecond.of(1));
    }

    /** Bumps the manual target speed down by 1 RPS. */
    public void decreaseShootRPS() {
        shootRPS = shootRPS.minus(RotationsPerSecond.of(1));
    }

    /**
     * The manual target speed set via
     * {@link #increaseShootRPS()}/{@link #decreaseShootRPS()}.
     */
    public AngularVelocity getShootRPS() {
        return shootRPS;
    }

    /**
     * Scales {@link ShooterConstants#MaxRPS} by a [-1, 1] axis input for manual
     * control.
     */
    public AngularVelocity getManualRPS(double axisInput) {
        return MaxRPS.times(axisInput);
    }

    /** The flywheel's measured angular velocity. */
    public AngularVelocity getCurrentRPS() {
        return m_motorLeft.getVelocity().getValue();
    }

    /** The flywheel's currently commanded angular velocity. */
    public AngularVelocity getTargetRPS() {
        return targetRPS;
    }

    @Override
    public void periodic() {
        DogLog.log("Shooter/State", shooterState);
        DogLog.log("Shooter/Primed", primed.getAsBoolean());
        DogLog.log("Shooter/ActualRPS", getCurrentRPS());
        DogLog.log("Shooter/TargetRPS", targetRPS);
        DogLog.log("Shooter/ManualRPS", shootRPS);
    }

    /** Returns a command that primes the shooter to the default prime RPS. */
    public Command prime() {
        return run(() -> {
            shoot(DefaultPrimeRPS);
            shooterState = ShooterState.PRIMING;
        });
    }

    /**
     * Returns a command that primes then shoots at whatever RPS {@code rpsSupplier}
     * reports.
     */
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
     * Returns a command that shoots at an RPS computed from the robot's pose to
     * {@code target}.
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

            return ShooterPhysics.calculateRPS(shotGroundDistance, target.getMeasureZ());
        };

        return new ShootCommand(
                this, kicker, indexer,
                rpsSupplier,
                primingState, shootingState);
    }

    /**
     * Returns a command that auto-aims and shoots into the hub based on the robot's
     * live pose.
     */
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

    /**
     * Returns a command that auto-aims and shoots a pass based on the robot's live
     * pose.
     */
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
