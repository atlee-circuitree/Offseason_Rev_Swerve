// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;

public class ChangeAngleCommand extends CommandBase {
  /** Creates a new RunFeederCommand. */
 
  FeederSubsystem m_FeederSubsystem;
  double m_SetPoint;
 
  public ChangeAngleCommand(double SetPoint, FeederSubsystem feederSubsystem) {
 
    m_FeederSubsystem = feederSubsystem;
    m_SetPoint = SetPoint;
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_FeederSubsystem.UpdateSetPoint(m_SetPoint);
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
