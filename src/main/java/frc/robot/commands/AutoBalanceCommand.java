// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalanceCommand extends CommandBase {
  /** Creates a new AutoBalance. */

  private DriveSubsystem drivetrain = new DriveSubsystem();
 
  public AutoBalanceCommand(DriveSubsystem dt) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = dt;
 
    addRequirements(drivetrain);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
    drivetrain.AutoBalance();

  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    drivetrain.KillDrive();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
      return false;

  }
}