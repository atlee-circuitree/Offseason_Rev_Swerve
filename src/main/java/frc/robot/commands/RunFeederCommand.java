// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.FeederSubsystem;

public class RunFeederCommand extends CommandBase {
  /** Creates a new RunFeederCommand. */
 
  double m_speed;
  FeederSubsystem m_FeederSubsystem;

  public RunFeederCommand(double speed, FeederSubsystem feederSubsystem) {
    
    m_speed = speed;
    m_FeederSubsystem = feederSubsystem;

    addRequirements(feederSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_FeederSubsystem.RunFeeder(m_speed);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_FeederSubsystem.RunFeeder(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
