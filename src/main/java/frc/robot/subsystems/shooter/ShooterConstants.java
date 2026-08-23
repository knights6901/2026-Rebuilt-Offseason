package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;

public final class ShooterConstants {
    /** The CAN ID of the left shooter motor. */
    public final static int LeftMotorId = 35;
    /** The CAN ID of the right shooter motor. */
    public final static int RightMotorId = 36;

    /** The distance from the center of the robot to the shooter (horizontally). */
    public final static Distance CenterToShooter = Inches.of(8);

    /** The maximum rotations per second that the shooter can achieve. */
    public final static AngularVelocity MaxRPS = RotationsPerSecond.of(80);
    /**
     * The default rotations per second of the shooter to shoot a ball (tested
     * experimentally).
     */
    public final static AngularVelocity DefaultRPS = RotationsPerSecond.of(30);

    /**
     * The tolerance for determining whether the shooter is "primed" and ready to
     * shoot.
     */
    public final static AngularVelocity PrimingTolerance = RotationsPerSecond.of(3);

    /** The default prime RPS for the shooter. */
    public final static AngularVelocity DefaultPrimeRPS = RotationsPerSecond.of(40);

    /** The PID and feedforward settings for the shooter motors. */
    public final static Slot0Configs Gains = new Slot0Configs()
            .withKP(0.36901).withKI(0).withKD(0.0085)
            .withKS(0).withKV(0.119);

    /** The strength of gravity (9.81 m/s²). */
    public final static LinearAcceleration G = MetersPerSecondPerSecond.of(9.81);

    /** The vertical position of the ball exit point from the shooter. */
    public final static Distance BallExtakeHeight = Meters.of(0.432);
    /** The angle at which the shooter is mounted above the horizontal plane. */
    public final static Angle Pitch = Degrees.of(76);

    /**
     * The scaling constant to correct for damping in the shooter mechanism when the
     * robot is "far" from the hub.
     */
    public final static double DampingFarCoefficient = 1.75;
    /**
     * The scaling constant to correct for damping in the shooter mechanism when the
     * robot is "near" from the hub.
     */
    public final static double DampingNearCoefficient = 2.7;

    /** The maximum distance to be considered "near" to the hub. */
    public final static Distance NearHubDistance = Meters.of(2);

    /** The radius of the shooter flywheel, used to convert exit velocity to RPS. */
    public final static Distance WheelRadius = Meters.of(0.051);

    /** The complete motor configuration for the shooter system. */
    public static final TalonFXConfiguration MotorConfig = new TalonFXConfiguration()
            .withSlot0(ShooterConstants.Gains)
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(40))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(60))
                    .withSupplyCurrentLimitEnable(true));
}
