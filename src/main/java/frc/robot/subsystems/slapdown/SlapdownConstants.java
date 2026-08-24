package frc.robot.subsystems.slapdown;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public final class SlapdownConstants {
    /** The CAN ID of the slapdown motor. */
    public final static int MotorId = 31;

    /** The position to lower the slapdown to when intaking a ball. */
    public final static Angle IntakePosition = Rotations.of(62);
    /** The default home position of slapdown system. */
    public final static Angle HomePosition = Rotations.of(0);
    /**
     * The tolerance for determining whether the slapdown is in the deployed
     * position.
     */
    public final static Angle PositionTolerance = Rotations.of(1.0);

    /**
     * The arm's elevation above horizontal when the motor is at
     * {@link #HomePosition}: folded straight up, inside the frame perimeter.
     * Because this is vertical, gravity puts almost no load on the retracted arm.
     */
    public final static Angle HomeAngle = Degrees.of(90);

    /**
     * The arm's elevation above horizontal when the motor is at
     * {@link #IntakePosition}: swung out to just past horizontal.
     */
    public final static Angle DeployedAngle = Degrees.of(-15);

    /**
     * The nominal gear ratio of the slapdown system. Nothing computes with this:
     * the build has enough slack that {@link #IntakePosition} had to be tuned well
     * past what this ratio predicts, so the motor-to-arm mapping is taken from the
     * calibrated endpoints instead. See {@link #EffectiveGearRatio}.
     */
    public final static double GearRatio = 80.0;

    /**
     * Motor rotations per rotation of the arm, implied by the calibrated
     * endpoints. Substantially larger than {@link #GearRatio} because
     * {@link #IntakePosition} also absorbs the mechanism's slack. The sim is built
     * on this so that it reproduces what the encoder actually reports.
     */
    public final static double EffectiveGearRatio = Math.abs(
            IntakePosition.minus(HomePosition).in(Rotations)
                    / HomeAngle.minus(DeployedAngle).in(Rotations));

    /**
     * The translation from the robot origin (centered on the drivebase, on the
     * floor plane) to the slapdown's hinge axis, used to place the arm's
     * AdvantageScope component model.
     */
    public final static Translation3d PivotOffset = new Translation3d(0.450, 0.013, 0.228);

    /**
     * Which side of the robot the arm swings out toward: {@code +1} for the front
     * ({@code +X}), {@code -1} for the back. Flip it if the arm ends up on the
     * wrong side in the visualization.
     */
    public final static double DeployDirection = 1.0;

    /**
     * The length of the arm, hinge axis to roller axis, measured off the
     * component model. Used only by the sim.
     */
    public final static Distance ArmLength = Meters.of(0.345);

    /** The mass of the arm. Used only by the sim. */
    public final static Mass ArmMass = Kilograms.of(3.0);

    /** The motor driving the arm, as modelled by the sim. */
    public final static DCMotor Gearbox = DCMotor.getKrakenX60(1);

    /**
     * Whether the sim pulls the arm down under gravity. Turn it off to get a
     * mechanism that only ever moves when commanded, which is easier to calibrate
     * the visualization against.
     */
    public final static boolean SimulateGravity = true;

    /**
     * PID/feedforward gains for driving down to the intake position. Needs tuning
     * after intake modification.
     */
    public final static Slot0Configs DownGains = new Slot0Configs()
            .withKP(0.4).withKI(0).withKD(0.1)
            .withKS(0).withKV(1.3);

    /**
     * PID/feedforward gains for driving up to the home position. Needs heavy
     * tuning.
     */
    public final static Slot1Configs UpGains = new Slot1Configs()
            .withKP(0).withKI(0).withKD(0)
            .withKS(0).withKV(0);

    /** The complete motor configuration for the slapdown system. */
    public final static TalonFXConfiguration MotorConfig = new TalonFXConfiguration()
            .withSlot0(SlapdownConstants.DownGains)
            .withSlot1(SlapdownConstants.UpGains)
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(40))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(60))
                    .withSupplyCurrentLimitEnable(true));
}
