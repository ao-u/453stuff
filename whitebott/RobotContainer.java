// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.TankDriveCmd;
import frc.robot.subsystems.TankDriveSubsystem;





public class RobotContainer {
  private final TankDriveSubsystem m_tankDriveSubsystem = new TankDriveSubsystem();
  private final TankDriveCmd m_tankDriveCmd = new TankDriveCmd(m_tankDriveSubsystem);
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final ManualControlArmCmd m_mControlArmCmd = new ManualControlArmCmd(m_armSubsystem);
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_tankDriveSubsystem.setDefaultCommand(m_tankDriveCmd);
    m_armSubsystem.setDefaultCommand(m_mControlArmCmd);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
