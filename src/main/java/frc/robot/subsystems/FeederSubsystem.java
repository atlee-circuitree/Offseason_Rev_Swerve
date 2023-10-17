// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {
  /** Creates a new FeederSubsystem. */

    double ArmAngle;

    CANSparkMax FrontFeedMotor;
    TalonFX LeftFeedMotor;
    TalonFX RightFeedMotor;
    CANSparkMax AngleMotor;
    SparkMaxAbsoluteEncoder AngleEncoder;

  public FeederSubsystem() {


    FrontFeedMotor = new CANSparkMax(Constants.DriveConstants.kFrontFeederCanId, MotorType.kBrushless);
    AngleMotor = new CANSparkMax(Constants.DriveConstants.kAngleCanId, MotorType.kBrushless);
    LeftFeedMotor = new TalonFX(Constants.DriveConstants.kLeftFeederCanId);
    RightFeedMotor = new TalonFX(Constants.DriveConstants.kRightFeederCanId);
 
    AngleEncoder = AngleMotor.getAbsoluteEncoder(Type.kDutyCycle);
    
  }

  public void RunFeeder(double speed) {

    FrontFeedMotor.set(speed);
    LeftFeedMotor.set(TalonFXControlMode.PercentOutput, speed);
    RightFeedMotor.set(TalonFXControlMode.PercentOutput, -speed);

  }

  public void RunAngle(double speed) {

    if ( ( AngleEncoder.getPosition() > .84 && speed < 0) || (AngleEncoder.getPosition() < .6181 && speed > 0)) {
 
      AngleMotor.set(0);

    } else {

      AngleMotor.set(speed);

    }
 
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("ArmEncodserValue", AngleEncoder.getPosition());

    // This method will be called once per scheduler run
  }
}
