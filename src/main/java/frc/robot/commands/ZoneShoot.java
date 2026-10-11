package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

import frc.robot.Constants.GameConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.slapdown.Slapdown;
import frc.robot.subsystems.vision.Vision;

/**
 * Aims and shoots at whatever makes sense for where the robot is when the
 * command starts. Inside our alliance zone it aligns to the hub and shoots into
 * it. Anywhere else it aligns straight back toward our alliance wall and
 * passes. Does nothing outside of teleop.
 */
public class ZoneShoot extends ConditionalCommand {
    public ZoneShoot(Drive drivetrain, Vision vision, Shooter shooter, Kicker kicker, Slapdown slapdown) {
        super(
                new ConditionalCommand(
                        new AlignToHub(drivetrain, vision)
                                .andThen(shooter.autoAimShoot(drivetrain::getPose, kicker, slapdown)),
                        new AlignShooter(
                                drivetrain,
                                () -> GameConstants.getAllianceWallDirection()
                                        .minus(drivetrain.getPose().getRotation()))
                                .andThen(shooter.passShoot(drivetrain::getPose, kicker, slapdown)),
                        () -> GameConstants.isInAllianceZone(drivetrain.getPose())),
                Commands.none(),
                DriverStation::isTeleop);
    }
}
