// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.drive.DriveConstants.kMaxAngularRate;
import static frc.robot.subsystems.drive.DriveConstants.kMaxSpeed;
import static frc.robot.subsystems.drive.DriveConstants.kRotationDeadband;
import static frc.robot.subsystems.drive.DriveConstants.kTranslationDeadband;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.Operator;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.slapdown.Slapdown;
import frc.robot.subsystems.led.LED;

public class RobotContainer {
    private final Drive drivetrain;
    private final Vision vision;
    private final Indexer indexer;
    private final Kicker kicker;
    private final Slapdown slapdown;
    private final LED led;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(kMaxSpeed * kTranslationDeadband)
            .withRotationalDeadband(kMaxAngularRate * kRotationDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final CommandXboxController driver = new CommandXboxController(Operator.kDriverControllerPort);

    public RobotContainer() {
        drivetrain = TunerConstants.createDrivetrain();
        vision = new Vision(drivetrain);
        indexer = new Indexer();
        kicker = new Kicker();
        slapdown = new Slapdown();
        led = new LED(drivetrain);

        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(driver.getLeftY() * kMaxSpeed)
                        .withVelocityY(driver.getLeftX() * kMaxSpeed)
                        .withRotationalRate(-driver.getRightX() * kMaxAngularRate)));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
