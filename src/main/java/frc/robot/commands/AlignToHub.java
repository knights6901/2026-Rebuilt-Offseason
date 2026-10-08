package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GameConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.vision.Vision;

/**
 * Rotates in place until the robot is aimed at our alliance's hub, then ends.
 */
public class AlignToHub extends Command {
    private static final double kThetaP = 4.0;
    private static final Angle kThetaTolerance = Degrees.of(2);

    /**
     * The side of the robot to point at the hub. This is the back, which is where
     * the camera looks, so the hub tags are in view once we're aligned.
     */
    private static final Rotation2d kAimDirection = Rotation2d.k180deg;

    private final Drive drivetrain;
    private final Vision vision;

    private final RobotCentric request = new RobotCentric().withDriveRequestType(DriveRequestType.Velocity);

    /** How far the robot still has to turn, CCW-positive, in radians. */
    private double thetaError;

    public AlignToHub(Drive drivetrain, Vision vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        Rotation2d hubBearing = vision.getHubBearing().orElseGet(this::getHubBearingFromPose);
        thetaError = hubBearing.minus(kAimDirection).getRadians();

        double omega = MathUtil.clamp(
                kThetaP * thetaError,
                -DriveConstants.kMaxAngularRate,
                DriveConstants.kMaxAngularRate);

        drivetrain.setControl(request.withRotationalRate(omega));
    }

    /** The robot-relative bearing to the hub according to the drivetrain pose. */
    private Rotation2d getHubBearingFromPose() {
        Pose2d pose = drivetrain.getPose();

        return GameConstants.getHubLocation()
                .minus(pose.getTranslation())
                .getAngle()
                .minus(pose.getRotation());
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
