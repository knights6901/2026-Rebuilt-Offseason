package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;

public final class KickerConstants {
        /** The CAN ID of the kicker motor. */
        public final static int MotorId = 24;

        /** The default speed of the kicker wheels. */
        public final static AngularVelocity KickerPower = RotationsPerSecond.of(75);

        /** PID and feedforward gains for the kicker motor. */
        public final static Slot0Configs Gains = new Slot0Configs()
                        .withKP(1.2).withKI(0).withKD(0)
                        .withKS(0.5).withKV(0.285);

        /** The complete motor configuration for the kicker system. */
        public final static TalonFXConfiguration MotorConfig = new TalonFXConfiguration()
                        .withSlot0(Gains)
                        .withMotorOutput(new MotorOutputConfigs()
                                        .withNeutralMode(NeutralModeValue.Coast)
                                        .withInverted(InvertedValue.Clockwise_Positive))
                        .withCurrentLimits(new CurrentLimitsConfigs()
                                        .withStatorCurrentLimit(Amps.of(60))
                                        .withStatorCurrentLimitEnable(true)
                                        .withSupplyCurrentLimit(Amps.of(80))
                                        .withSupplyCurrentLimitEnable(true));
}
