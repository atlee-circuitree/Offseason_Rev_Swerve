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
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
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

    RelativeEncoder AngleEncoder;

    PIDController AnglePID;
 
  public FeederSubsystem() {


    FrontFeedMotor = new CANSparkMax(Constants.DriveConstants.kFrontFeederCanId, MotorType.kBrushless);
    AngleMotor = new CANSparkMax(Constants.DriveConstants.kLeftFeederCanId, MotorType.kBrushless);
    LeftFeedMotor = new TalonFX(Constants.DriveConstants.kLeftFeederCanId);
    RightFeedMotor = new TalonFX(Constants.DriveConstants.kRightFeederCanId);
 
    AngleEncoder = AngleMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);

    AnglePID = new PIDController(.01, 0, 0);
    
  }

  public void RunFeeder(double speed) {

    FrontFeedMotor.set(speed);
    LeftFeedMotor.set(TalonFXControlMode.PercentOutput, speed);
    RightFeedMotor.set(TalonFXControlMode.PercentOutput, -speed);

  }

  public void RunAngle(double speed) {

    if (speed > 0) {

      AnglePID.setSetpoint(.68);

      if (AngleEncoder.getPosition() < .68) {
 
        System.out.println("Top limit hit, trying to go up " + speed);
        AngleMotor.set(0);
  
      } else {

        AngleMotor.set(AnglePID.calculate(speed, AnglePID.getSetpoint()));

      }

    }

    if (speed < 0) {

      AnglePID.setSetpoint(.9);

      if (AngleEncoder.getPosition() < .9) {
 
        System.out.println("Bottom limit hit, trying to go up " + speed);
        AngleMotor.set(0);
  
      } else {

        AngleMotor.set(AnglePID.calculate(speed, AnglePID.getSetpoint()));

      }

    }

    if (speed == 0) {

      AngleMotor.set(0);

    }
  
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Arm Encoder Value", AngleEncoder.getPosition());

    // This method will be called once per scheduler run
  }
}
