package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
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
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

public final class ShooterConstants {
        /** The CAN ID of the left shooter motor. */
        public final static int LeftMotorId = 23;
        /** The CAN ID of the right shooter motor. */
        public final static int RightMotorId = 21;

        /**
         * The horizontal distance from the center of the robot to the center of the
         * shooter drum.
         */
        public final static Distance CenterToShooter = Inches.of(10);

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
        public final static AngularVelocity PrimingTolerance = RotationsPerSecond.of(5);

        /** The default prime RPS for the shooter. */
        public final static AngularVelocity DefaultPrimeRPS = RotationsPerSecond.of(40);

        /** The strength of gravity (9.81 m/s²). */
        public final static LinearAcceleration G = MetersPerSecondPerSecond.of(9.81);

        /** The vertical position of the ball exit point from the shooter. */
        public final static Distance BallExtakeHeight = Inches.of(20.5);
        /** The angle at which the shooter is mounted above the horizontal plane. */
        public final static Angle Pitch = Degrees.of(80);

        /** The radius of the shooter drum, used to convert exit velocity to RPS. */
        public final static Distance WheelRadius = Inches.of(2);

        /** The combined mass of the twelve flywheels that make up the drum. */
        public final static Mass FlywheelMass = Pounds.of(12);

        /**
         * The drum's moment of inertia, treating the flywheels as solid discs. Sets how
         * much the drum slows while it launches a ball.
         */
        public final static MomentOfInertia FlywheelInertia = KilogramSquareMeters.of(
                        0.5 * FlywheelMass.in(Kilograms) * Math.pow(WheelRadius.in(Meters), 2));

        /**
         * The fraction of the ideal exit speed a ball actually leaves with. Ideal is a
         * ball rolling between the drum and the hood without slipping, which carries it
         * out at half the drum's surface speed; slip and the energy spent squashing the
         * foam take it below that.
         *
         * <p>
         * This is the one number to calibrate on the robot. If shots fall short at
         * every distance, lower it; if they sail long, raise it.
         */
        public final static double ExitSpeedEfficiency = 0.85;

        /** The diameter of a FUEL, from the game manual. */
        public final static Distance FuelDiameter = Inches.of(5.91);

        /** The mass of a FUEL. The game manual allows 0.448 to 0.500 lb. */
        public final static Mass FuelMass = Pounds.of(0.474);

        /**
         * A FUEL's moment of inertia as a fraction of {@code mass * radius^2}. 2/5 is
         * a uniform solid sphere, which is what a foam ball is.
         */
        public final static double FuelInertiaFactor = 0.4;

        /** The drag coefficient of a FUEL: a sphere below its drag crisis. */
        public final static double FuelDragCoefficient = 0.47;

        /**
         * The Magnus coefficient of a FUEL from the topspin it leaves with: the drum
         * sits above the ball and the hood below, so the top of the ball is driven
         * forward. A ball that rolls out of the shooter has a surface speed equal to
         * its flight speed, where measured coefficients for spinning spheres flatten
         * out around this value. Set to zero to ignore spin.
         */
        public final static double FuelMagnusCoefficient = 0.25;

        /** The density of air at sea level and 15 C, in kilograms per cubic meter. */
        public final static double AirDensity = 1.225;

        /** The PID and feedforward settings for the shooter motors. */
        public final static Slot0Configs Gains = new Slot0Configs()
                        .withKP(0.8).withKI(0).withKD(0.025)
                        .withKS(0.25).withKV(0.122);

        /** The complete motor configuration for the shooter system. */
        public static final TalonFXConfiguration MotorConfig = new TalonFXConfiguration()
                        .withSlot0(ShooterConstants.Gains)
                        .withMotorOutput(new MotorOutputConfigs()
                                        .withNeutralMode(NeutralModeValue.Coast)
                                        .withInverted(InvertedValue.Clockwise_Positive))
                        .withCurrentLimits(new CurrentLimitsConfigs()
                                        .withStatorCurrentLimit(Amps.of(40))
                                        .withStatorCurrentLimitEnable(true)
                                        .withSupplyCurrentLimit(Amps.of(60))
                                        .withSupplyCurrentLimitEnable(true));
}
