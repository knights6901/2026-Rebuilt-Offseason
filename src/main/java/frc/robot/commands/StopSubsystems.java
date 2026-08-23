// Stop all shooter-related subsystems when needed
// Note: If the shooter stops, all related mechanisms should stop anyway

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;

/**
 * Stops all shooter-related subsystems.
 * 
 * <p>
 * This command disables the shooter, kicker, and intake subsystems. Use this
 * command to halt all shooting-related mechanisms at once. If you need more
 * fine-grained control where some subsystems continue while others stop,
 * consider
 * using alternative commands with more specific requirements.
 * 
 * <p>
 * Requires: {@link ShooterSubsystem}, {@link KickerSubsystem},
 * {@link IntakeSubsystem}, {@link IndexerSubsystem}
 */
public class StopSubsystems extends Command {
    private Shooter shooter;
    private Kicker kicker;
    private Intake intake;
    private Indexer indexer;

    /**
     * Constructs a StopSubsystemsCommand.
     *
     * @param shooter the shooter subsystem to stop
     * @param kicker  the kicker subsystem to stop
     * @param intake  the intake subsystem to stop
     * @param indexer the indexer subsystem to stop
     */
    public StopSubsystems(Shooter shooter, Kicker kicker, Intake intake,
            Indexer indexer) {
        this.shooter = shooter;
        this.kicker = kicker;
        this.intake = intake;
        this.indexer = indexer;
        addRequirements(shooter, kicker, intake, indexer);
    }

    /**
     * Immediately stops all controlled subsystems.
     */
    @Override
    public void execute() {
        shooter.stop();
        kicker.stop();
        intake.stop();
        indexer.stop();
    }

    /**
     * This command runs continuously until manually interrupted.
     *
     * @return {@code false} to run continuously
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}