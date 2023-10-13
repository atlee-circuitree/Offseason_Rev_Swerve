// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {
  /** Creates a new FeederSubsystem. */

    CANSparkMax FrontFeedMotor;
    TalonFX LeftFeedMotor;
    TalonFX RightFeedMotor;
    CANSparkMax AngleMotor;

  public FeederSubsystem() {

    FrontFeedMotor = new CANSparkMax(Constants.DriveConstants.kFrontFeederCanId, MotorType.kBrushless);
    AngleMotor = new CANSparkMax(Constants.DriveConstants.kLeftFeederCanId, MotorType.kBrushless);
    LeftFeedMotor = new TalonFX(Constants.DriveConstants.kLeftFeederCanId);
    RightFeedMotor = new TalonFX(Constants.DriveConstants.kRightFeederCanId);

  }

  public void RunFeeder(double speed) {

    FrontFeedMotor.set(speed);
    LeftFeedMotor.set(TalonFXControlMode.PercentOutput, speed);
    RightFeedMotor.set(TalonFXControlMode.PercentOutput, -speed);

  }

  public void RunAngle(double speed) {

    AngleMotor.set(speed);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
