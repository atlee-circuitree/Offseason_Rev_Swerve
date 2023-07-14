// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
   
  CANSparkMax LeftIntakeMotor;
  CANSparkMax RightIntakeMotor;
  CANSparkMax FrontIntakeMotor;

  public IntakeSubsystem() {

    LeftIntakeMotor = new CANSparkMax(Constants.DriveConstants.kLeftIntakeMotorCanId, MotorType.kBrushless);
    RightIntakeMotor = new CANSparkMax(Constants.DriveConstants.kRightIntakeMotorCanId, MotorType.kBrushless);
    FrontIntakeMotor = new CANSparkMax(Constants.DriveConstants.kFrontIntakeMotorCanId, MotorType.kBrushless);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
