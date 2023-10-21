// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
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
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
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

    PIDController AnglePID;

    AnalogInput DistanceSensor;

  public FeederSubsystem() {


    FrontFeedMotor = new CANSparkMax(Constants.DriveConstants.kFrontFeederCanId, MotorType.kBrushless);
    AngleMotor = new CANSparkMax(Constants.DriveConstants.kAngleCanId, MotorType.kBrushless);
    LeftFeedMotor = new TalonFX(Constants.DriveConstants.kLeftFeederCanId);
    RightFeedMotor = new TalonFX(Constants.DriveConstants.kRightFeederCanId);

    LeftFeedMotor.setNeutralMode(NeutralMode.Brake);
    RightFeedMotor.setNeutralMode(NeutralMode.Brake);
    FrontFeedMotor.setIdleMode(IdleMode.kBrake);
 
    AngleEncoder = AngleMotor.getAbsoluteEncoder(Type.kDutyCycle);

    DistanceSensor = new AnalogInput(0);
 
    AnglePID = new PIDController(.3, 0, 0);
 
  }

  public void RunFeeder(double speed) {
 
    if (DistanceSensor.getVoltage() < .3) {

      FrontFeedMotor.set(0);
      LeftFeedMotor.set(TalonFXControlMode.PercentOutput, 0);
      RightFeedMotor.set(TalonFXControlMode.PercentOutput, 0);

    } else {

      FrontFeedMotor.set(-speed);
      LeftFeedMotor.set(TalonFXControlMode.PercentOutput, speed);
      RightFeedMotor.set(TalonFXControlMode.PercentOutput, -speed);

    }

  }

  public double GetEncoder() {

    return AngleEncoder.getPosition();

  }

  public void UpdateSetPoint(double SetPoint) {

    AnglePID.setSetpoint(SetPoint);

  }

  public void MaintainAngle() {

    double Adjustment = 4;
    int IsNegative = 1;
    // 1 = True, -1 = False

    if (AnglePID.getSetpoint() == 0) {

      AnglePID.setSetpoint(AngleEncoder.getPosition());

    }
    
    double error = Math.abs(10 * (AngleEncoder.getPosition() - AnglePID.getSetpoint()));

    SmartDashboard.putNumber("Maintain Angle", AnglePID.getSetpoint());
    SmartDashboard.putNumber("Current Angle", AngleEncoder.getPosition());
 
    if (AngleEncoder.getPosition() > AnglePID.getSetpoint()) {
 
      System.out.println("Correcting Upward" + " " + AngleEncoder.getPosition() + " < " + AnglePID.getSetpoint());
      AngleMotor.set(error);
      
    } else if (AngleEncoder.getPosition() < AnglePID.getSetpoint()) {
 
      System.out.println("Correcting Downward" + " " + AngleEncoder.getPosition() + " > " + AnglePID.getSetpoint());
      AngleMotor.set(-error);

    } else {

      AngleMotor.set(0);

    }
 
    if (IsNegative * Adjustment * AnglePID.calculate(AngleEncoder.getPosition(), AnglePID.getSetpoint()) < 0 && AngleEncoder.getPosition() > .92) {

      AngleMotor.set(0);

    } else if (IsNegative * Adjustment * -AnglePID.calculate(AngleEncoder.getPosition(), AnglePID.getSetpoint()) > 0 && AngleEncoder.getPosition() < .62) {

      AngleMotor.set(0);

    }

  }

  public void RunAngle(double speed) {
 
    double Adjustment = 30;
  
    if (speed > 0) {

      AnglePID.setSetpoint(.68);

      if (AngleEncoder.getPosition() < .68) {
 
        System.out.println("Top limit hit, trying to go up " + Adjustment * AnglePID.calculate(AngleEncoder.getPosition(), AnglePID.getSetpoint()));
        AngleMotor.set(0);
  
      } else {
 
        System.out.println(Adjustment * AnglePID.calculate(AngleEncoder.getPosition(), AnglePID.getSetpoint()));
        AngleMotor.set(Adjustment * -AnglePID.calculate(AngleEncoder.getPosition(), AnglePID.getSetpoint()));
        //AngleMotor.set(speed);

      }

    }

    if (speed < 0) {

      AnglePID.setSetpoint(.92);

      if (AngleEncoder.getPosition() > .92) {
 
        System.out.println("Bottom limit hit, trying to go down " + Adjustment * AnglePID.calculate(AngleEncoder.getPosition(), AnglePID.getSetpoint()));
        //AngleMotor.set(AnglePID.calculate(speed, AnglePID.getSetpoint()));
        AngleMotor.set(0);
  
      } else {

        System.out.println(Adjustment * AnglePID.calculate(AngleEncoder.getPosition(), AnglePID.getSetpoint()));
        AngleMotor.set(Adjustment * -AnglePID.calculate(AngleEncoder.getPosition(), AnglePID.getSetpoint()));
        //AngleMotor.set(speed);

      }

    }

    if (speed == 0) {

      AngleMotor.set(0);

    }
  
  }

  public void RunAngle2(double speed) {

    AngleMotor.set(speed);

  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Arm Encoder Value", AngleEncoder.getPosition());
    SmartDashboard.putNumber("Distance", DistanceSensor.getVoltage() );
 
    // This method will be called once per scheduler run
  }
}
