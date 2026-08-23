package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class KickerConstants {
        /** The CAN ID of the kicker motor. */
        public final static int MotorId = 37;

        public final static double KickerPower = 0.8506901;

        /** The complete motor configuration for the kicker system. */
        public final static TalonFXConfiguration MotorConfig = new TalonFXConfiguration()
                        .withMotorOutput(new MotorOutputConfigs()
                                        .withNeutralMode(NeutralModeValue.Coast)
                                        .withInverted(InvertedValue.Clockwise_Positive))
                        .withCurrentLimits(new CurrentLimitsConfigs()
                                        .withStatorCurrentLimit(Amps.of(60))
                                        .withStatorCurrentLimitEnable(true)
                                        .withSupplyCurrentLimit(Amps.of(80))
                                        .withSupplyCurrentLimitEnable(true));
}