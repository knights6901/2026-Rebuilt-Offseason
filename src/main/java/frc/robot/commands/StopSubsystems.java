package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;

/** Stops the shooter, kicker, intake, and indexer all at once. Runs until interrupted. */
public class StopSubsystems extends ParallelCommandGroup {
    public StopSubsystems(Shooter shooter, Kicker kicker, Intake intake, Indexer indexer) {
        super(
                shooter.run(shooter::stop),
                kicker.stop(),
                intake.stop(),
                indexer.stop());
    }
}
