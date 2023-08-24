// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class RotateModulesCommand extends CommandBase {

  DriveSubsystem m_DriveSubsystem;
  Double m_Angle;

  public RotateModulesCommand(DriveSubsystem Drive, Double Angle) {
    
    m_DriveSubsystem = Drive;
    m_Angle = Angle;

    addRequirements(m_DriveSubsystem);

  }
 
  @Override
  public void initialize() {

    m_DriveSubsystem.setAllToAngle(m_Angle);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
