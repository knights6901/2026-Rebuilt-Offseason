// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain.Drivetrain;

public class RobotContainer {
  private static Drivetrain drivetrain = DrivetrainConstants.createDrivetrain();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(DrivetrainConstants.MaxSpeed * 0.1)
      .withRotationalDeadband(DrivetrainConstants.MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final CommandXboxController driver = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> drive
            .withVelocityX(driver.getLeftY() * DrivetrainConstants.MaxSpeed)
            .withVelocityY(driver.getLeftX() * DrivetrainConstants.MaxSpeed)
            .withRotationalRate(-driver.getRightX() * DrivetrainConstants.MaxAngularRate)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
