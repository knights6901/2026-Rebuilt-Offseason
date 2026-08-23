package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;

public final class IndexerConstants {
    /** The CAN ID of the indexer motor. */
    public final static int MotorId = 40;

    /** The target rotations per second for the indexer motor during operation. */
    public final static AngularVelocity Power = RotationsPerSecond.of(85);

    /** The complete motor configuration for the indexer system. */
    public final static TalonFXConfiguration MotorConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(40))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(60))
                    .withSupplyCurrentLimitEnable(true));
}
