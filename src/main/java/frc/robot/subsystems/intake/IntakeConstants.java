package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;

public final class IntakeConstants {
    /** The CAN ID of the intake motor. */
    public final static int MotorId = 32;

    /** The rotations per second for actively intaking balls. */
    public final static AngularVelocity IntakeRPS = RotationsPerSecond.of(85);

    /** The gear ratio of the intake system. */
    public final static double GearRatio = 9.0;

    /** PID and feedforward gains for the intake motor. */
    public final static Slot0Configs Gains = new Slot0Configs()
            .withKP(0.5).withKI(0).withKD(0)
            .withKS(0).withKV(0.15);

    /** The complete motor configuration for the intake system. */
    public final static TalonFXConfiguration MotorConfig = new TalonFXConfiguration()
            .withSlot0(IntakeConstants.Gains)
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(40))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(60))
                    .withSupplyCurrentLimitEnable(true));
}