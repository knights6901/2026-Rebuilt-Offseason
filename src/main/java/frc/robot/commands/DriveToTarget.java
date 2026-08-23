package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class DriveToTarget extends Command {
        private final Drive drivetrain;
        private final Supplier<SwerveRequest.FieldCentric> driverInput;

        // null = use driver input for that axis
        private final Supplier<Distance> targetX;
        private final Supplier<Distance> targetY;
        private final Supplier<Angle> targetTheta;

        // PID controllers for each axis, only used if the axis is being controlled by
        // the command
        private final PIDController xController = new PIDController(.5, 0, 0);
        private final PIDController yController = new PIDController(1, 0, 0);
        private final PIDController thetaController = new PIDController(2.5, 0, 0.0);

        private Angle thetaError;
        private Distance xError;
        private Distance yError;

        private final DoublePublisher thetaErrorPub = NetworkTableInstance.getDefault()
                        .getTable("DriveToTarget")
                        .getDoubleTopic("Error (theta)")
                        .publish();

        private final DoublePublisher xErrorPub = NetworkTableInstance.getDefault()
                        .getTable("DriveToTarget")
                        .getDoubleTopic("Error (x)")
                        .publish();

        private final DoublePublisher yErrorPub = NetworkTableInstance.getDefault()
                        .getTable("DriveToTarget")
                        .getDoubleTopic("Error (y)")
                        .publish();

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

                thetaController.setTolerance(Degrees.of(5).in(Radians));
        }

        @Override
        public void execute() {
                Pose2d currentPose = drivetrain.getPose();
                FieldCentric input = driverInput.get();

                double vX = targetX != null ? xController.calculate(currentPose.getX(), targetX.get().in(Meters))
                                : input.VelocityX;

                double vY = targetY != null ? yController.calculate(currentPose.getY(), targetY.get().in(Meters))
                                : input.VelocityY;

                double omega = targetTheta != null
                                ? thetaController.calculate(currentPose.getRotation().getRadians(),
                                                targetTheta.get().in(Radians))
                                : input.RotationalRate;

                drivetrain.setControl(input.withVelocityX(vX).withVelocityY(vY).withRotationalRate(omega));

                xError = targetX != null ? targetX.get().minus(currentPose.getMeasureX()) : Meters.of(0);
                yError = targetY != null ? targetY.get().minus(currentPose.getMeasureY()) : Meters.of(0);
                thetaError = targetTheta != null
                                ? targetTheta.get().minus(currentPose.getRotation().getMeasure())
                                : Degrees.of(0);

                thetaErrorPub.set(thetaError.in(Degrees));
                xErrorPub.set(xError.in(Meters));
                yErrorPub.set(yError.in(Meters));
        }

        @Override
        public boolean isFinished() {
                boolean xComplete = targetX == null || xController.atSetpoint();
                boolean yComplete = targetY == null || yController.atSetpoint();
                boolean thetaComplete = targetTheta == null || thetaController.atSetpoint();

                return xComplete && yComplete && thetaComplete;
        }
}