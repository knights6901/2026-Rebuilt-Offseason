package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;

/**
 * Spins up the shooter to a priming speed to prepare it for shooting game
 * pieces, then executes the shooting sequence once the shooter is primed.
 */
public class ShootCommand extends SequentialCommandGroup {
    public ShootCommand(
            Shooter shooter,
            Kicker kicker,
            Indexer indexer,
            Supplier<AngularVelocity> rpsSupplier,
            ShooterState primingState,
            ShooterState shootingState) {
        super(
                new RunCommand(() -> {
                    shooter.shoot(rpsSupplier.get());
                    shooter.shooterState = primingState;
                }, shooter).until(shooter.primed),
                new ParallelCommandGroup(
                        indexer.enable(),
                        kicker.kick(),
                        new RunCommand(() -> shooter.shooterState = shootingState, shooter)));
    }
}
