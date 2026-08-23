package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

/**
 * Drives toward a target pose. Each axis (x, y, theta) is independently
 * either PID-controlled toward its target or, if that target supplier is
 * {@code null}, left under driver joystick control.
 */
public class DriveToTarget extends Command {
    private static final double kXP = 0.5;
    private static final double kYP = 1.0;
    private static final double kThetaP = 2.5;
    private static final Angle kThetaTolerance = Degrees.of(5);

    private final Drive drivetrain;
    private final Supplier<FieldCentric> driverInput;

    /** Null means that axis is left under driver control. */
    private final Supplier<Distance> targetX;
    private final Supplier<Distance> targetY;
    private final Supplier<Angle> targetTheta;

    private final PIDController xController = new PIDController(kXP, 0, 0);
    private final PIDController yController = new PIDController(kYP, 0, 0);
    private final PIDController thetaController = new PIDController(kThetaP, 0, 0);

    public DriveToTarget(
            Drive drivetrain,
            Supplier<FieldCentric> driverInput,
            Supplier<Distance> targetX,
            Supplier<Distance> targetY,
            Supplier<Angle> targetTheta) {
        this.drivetrain = drivetrain;
        this.driverInput = driverInput;
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetTheta = targetTheta;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        xController.reset();
        yController.reset();
        thetaController.reset();
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(kThetaTolerance.in(Radians));
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getPose();
        FieldCentric input = driverInput.get();

        double vX = targetX != null
                ? xController.calculate(currentPose.getX(), targetX.get().in(Meters))
                : input.VelocityX;
        double vY = targetY != null
                ? yController.calculate(currentPose.getY(), targetY.get().in(Meters))
                : input.VelocityY;
        double omega = targetTheta != null
                ? thetaController.calculate(currentPose.getRotation().getRadians(), targetTheta.get().in(Radians))
                : input.RotationalRate;

        drivetrain.setControl(input.withVelocityX(vX).withVelocityY(vY).withRotationalRate(omega));

        DogLog.log("DriveToTarget/ErrorX", targetX != null ? xController.getError() : 0, Meters);
        DogLog.log("DriveToTarget/ErrorY", targetY != null ? yController.getError() : 0, Meters);
        DogLog.log("DriveToTarget/ErrorTheta", targetTheta != null ? thetaController.getError() : 0, Radians);
    }

    @Override
    public boolean isFinished() {
        boolean xComplete = targetX == null || xController.atSetpoint();
        boolean yComplete = targetY == null || yController.atSetpoint();
        boolean thetaComplete = targetTheta == null || thetaController.atSetpoint();

        return xComplete && yComplete && thetaComplete;
    }
}
