package frc.robot.subsystems.slapdown;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;

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

    /** The gear ratio of the slapdown system. */
    public final static double GearRatio = 81.0;

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
