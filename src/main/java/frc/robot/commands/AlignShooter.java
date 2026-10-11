package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;

/**
 * Rotates in place until the shooter is aimed along a robot-relative bearing,
 * then ends.
 */
public class AlignShooter extends Command {
    private static final double kThetaP = 4.0;
    private static final Angle kThetaTolerance = Degrees.of(2);

    /** The side of the robot the shooter fires out of. */
    private static final Rotation2d kAimDirection = Rotation2d.k180deg;

    private final Drive drivetrain;

    /** The robot-relative bearing to aim along, CCW-positive from the front. */
    private final Supplier<Rotation2d> targetBearing;

    private final RobotCentric request = new RobotCentric().withDriveRequestType(DriveRequestType.Velocity);

    /** How far the robot still has to turn, CCW-positive, in radians. */
    private double thetaError;

    public AlignShooter(Drive drivetrain, Supplier<Rotation2d> targetBearing) {
        this.drivetrain = drivetrain;
        this.targetBearing = targetBearing;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        thetaError = targetBearing.get().minus(kAimDirection).getRadians();

        double omega = MathUtil.clamp(
                kThetaP * thetaError,
                -DriveConstants.kMaxAngularRate,
                DriveConstants.kMaxAngularRate);

        drivetrain.setControl(request.withRotationalRate(omega));
    }

    @Override
    public boolean isFinished() {
        return Math.abs(thetaError) < kThetaTolerance.in(Radians);
    }

    @Override
    public void end(boolean interrupted) {
        /* The drivetrain keeps applying its last request, so stop it explicitly. */
        drivetrain.setControl(request.withRotationalRate(0));
    }
}
