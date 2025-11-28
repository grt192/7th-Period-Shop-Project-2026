// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.OuttakeSubsystem;

public class RobotContainer {
  private final OuttakeSubsystem m_outtakeSubsystem = new OuttakeSubsystem();

  private final CommandPS5Controller m_driverController = new CommandPS5Controller(
      OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();

    m_outtakeSubsystem.setDefaultCommand(
        m_outtakeSubsystem.VelocityControl(() -> m_driverController.getL2Axis(), () -> m_driverController.getR2Axis()));
  }

  private void configureBindings() {
    m_driverController.povUp().onTrue(m_outtakeSubsystem.stepUp());
    m_driverController.povDown().onTrue(m_outtakeSubsystem.stepDown());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
