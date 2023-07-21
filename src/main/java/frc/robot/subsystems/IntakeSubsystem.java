// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
   
  // Define Motors
  CANSparkMax LeftIntakeMotor;
  CANSparkMax RightIntakeMotor;
  CANSparkMax FrontIntakeMotor;

  public IntakeSubsystem() {

    // Set Motor IDs
    LeftIntakeMotor = new CANSparkMax(Constants.DriveConstants.kLeftIntakeMotorCanId, MotorType.kBrushless);
    RightIntakeMotor = new CANSparkMax(Constants.DriveConstants.kRightIntakeMotorCanId, MotorType.kBrushless);
    FrontIntakeMotor = new CANSparkMax(Constants.DriveConstants.kFrontIntakeMotorCanId, MotorType.kBrushless);

    // Set Idle Mode
    LeftIntakeMotor.setIdleMode(IdleMode.kCoast);
    RightIntakeMotor.setIdleMode(IdleMode.kCoast);
    FrontIntakeMotor.setIdleMode(IdleMode.kCoast);

  }

  @Override
  public void periodic() {
     

    
  }
}
