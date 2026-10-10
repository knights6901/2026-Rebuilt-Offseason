// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.drive.DriveConstants.kMaxAngularRate;
import static frc.robot.subsystems.drive.DriveConstants.kMaxSpeed;
import static frc.robot.subsystems.drive.DriveConstants.kRotationDeadband;
import static frc.robot.subsystems.drive.DriveConstants.kTranslationDeadband;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.Operator;
import frc.robot.commands.AlignToHub;
import frc.robot.commands.StopSubsystems;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.slapdown.Slapdown;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LEDConstants;

public class RobotContainer {
    private final Drive drivetrain;
    private final Vision vision;
    private final Intake intake;
    private final Kicker kicker;
    private final LED led;
    private final Shooter shooter;
    private final Slapdown slapdown;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(kMaxSpeed * kTranslationDeadband)
            .withRotationalDeadband(kMaxAngularRate * kRotationDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final CommandXboxController driver = new CommandXboxController(Operator.kDriverControllerPort);
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        drivetrain = TunerConstants.createDrivetrain();
        vision = new Vision(drivetrain);
        intake = new Intake();
        kicker = new Kicker();
        led = new LED(drivetrain);
        shooter = new Shooter();
        slapdown = new Slapdown();

        autoChooser = AutoBuilder.buildAutoChooser("zero");
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
        configureDefaultCommands();
        configurePathPlannerCommands();
        configureMirroredAutons();
    }

    private void configureBindings() {
        driver.a().onTrue(slapdown.slapdown());
        driver.b().onTrue(slapdown.retractSlapdown());
        driver.leftTrigger().whileTrue(intake.intake());

        driver.leftTrigger().whileTrue(intake.intake());
        driver.leftBumper().whileTrue(intake.outtake());

        driver.rightTrigger().whileTrue(shooter.manuallyShoot(() -> RotationsPerSecond.of(65), kicker));

        driver.x().onTrue(new InstantCommand(() -> slapdown.zeroSlapdownPosition(), slapdown));

        driver.rightBumper().whileTrue(new AlignToHub(drivetrain, vision));

        /*
         * Heading fallback for when no multi-tag solve has set it yet: point the robot
         * straight away from the driver and press to make that "forward".
         */
        driver.start().onTrue(drivetrain.runOnce(() -> {
            drivetrain.seedFieldCentric();
            vision.resetEstimatorHeading();
        }));

        // driver.rightBumper().whileTrue(shooter.shootCommand(RotationsPerSecond.of(75)));
        // driver.rightTrigger().whileTrue(shooter.autoAimShoot(drivetrain::getPose,
        // kicker, indexer));

        /*
         * Panic switch: hold to shut vision out of the pose estimator entirely, in case
         * a bad estimate starts dragging the pose around mid-match. Deliberately has no
         * subsystem requirement, so it cannot cancel the drive default command.
         */

        driver.back().whileTrue(
                Commands.startEnd(
                        () -> vision.setFusionEnabled(false),
                        () -> vision.setFusionEnabled(true))
                        .ignoringDisable(true));
    }

    private void configureDefaultCommands() {
        led.setDefaultCommand(led.runPattern(LEDConstants.ScrollRainbowPattern));
        intake.setDefaultCommand(intake.stop());
        shooter.setDefaultCommand(shooter.neutralOut());
        kicker.setDefaultCommand(kicker.stop());

        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-driver.getLeftY() * kMaxSpeed)
                        .withVelocityY(-driver.getLeftX() * kMaxSpeed)
                        .withRotationalRate(-driver.getRightX() * kMaxAngularRate)));
    }

    private void configurePathPlannerCommands() {
        NamedCommands.registerCommand("stopSubsystems",
                        new StopSubsystems(shooter, kicker, intake));

        NamedCommands.registerCommand("autoAimShoot",
                        shooter.autoAimShoot(drivetrain::getPose, kicker));

        NamedCommands.registerCommand("fiftyRPSShoot",
                        shooter.manuallyShoot(() -> RotationsPerSecond.of(50), kicker));

        NamedCommands.registerCommand("primeShooter", shooter.prime().withTimeout(Seconds.of(3)));
        NamedCommands.registerCommand("stopShooter",
                        new InstantCommand(() -> {
                                shooter.stop();
                                kicker.stop();
                        }, shooter, kicker));

        NamedCommands.registerCommand("intake", intake.intake());
        NamedCommands.registerCommand("stopIntake", intake.stop());

        NamedCommands.registerCommand("rotateToHub",
                        new AlignToHub(drivetrain, vision));

        NamedCommands.registerCommand("slapdownTrigger", slapdown.slapdown());
        NamedCommands.registerCommand("slapdownRetract", slapdown.retractSlapdown());
    }

    private void configureMirroredAutons() {
        // mirrored left autos for right side
        autoChooser.addOption("sam_rightHS", new PathPlannerAuto("sam_leftHS", true));
        autoChooser.addOption("sam_rightDoubleHS", new PathPlannerAuto("sam_leftDoubleHS", true));
    }


    /**
     * Returns the autonomous command selected from the SmartDashboard chooser.
     *
     * @return the selected autonomous {@link Command}
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
