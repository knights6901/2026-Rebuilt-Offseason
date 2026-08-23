package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public final class LEDConstants {
    /** The PWM port that the led bus is connected to the RIO on. */
    public static final int Port = 1;
    /** The length (in number of LED connections on the strip). */
    public static final int Length = 186;
    public static final int RightLength = 19;
    public static final int LeftLength = 36;
    public static final int MiddleLength = Length - RightLength - LeftLength;
    // RIGHT MIDDLE PART: 36
    // LEFT MIDDLE PART: 19
    // MIDDLE/Entire Robot: the rest
    public static final LEDPattern Off = LEDPattern.solid(Color.kBlack);
    public static final LEDPattern Red = LEDPattern.solid(Color.kRed);
    public static final LEDPattern Purple = LEDPattern.solid(Color.kPurple);

    public static final LEDPattern FlashingPurple = Purple.breathe(Seconds.of(0.5));

    public static final LEDPattern RainbowPattern = LEDPattern
            .rainbow(255, 128);
    public static final LEDPattern ScrollRainbowPattern = RainbowPattern.scrollAtRelativeSpeed(
            Percent.per(Second).of(120));
}
