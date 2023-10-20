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
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
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
    SparkMaxPIDController AnglePID;

  public FeederSubsystem() {


    FrontFeedMotor = new CANSparkMax(Constants.DriveConstants.kFrontFeederCanId, MotorType.kBrushless);
    AngleMotor = new CANSparkMax(Constants.DriveConstants.kAngleCanId, MotorType.kBrushless);
    LeftFeedMotor = new TalonFX(Constants.DriveConstants.kLeftFeederCanId);
    RightFeedMotor = new TalonFX(Constants.DriveConstants.kRightFeederCanId);
 
    AngleEncoder = AngleMotor.getAbsoluteEncoder(Type.kDutyCycle);
    /*
    AnglePID = AngleMotor.getPIDController();

    AnglePID.setP(5e-5);
    AnglePID.setI(1e-6);
    AnglePID.setD(0);
    AnglePID.setIZone(0);
    AnglePID.setFF(0.0001);
    AnglePID.setOutputRange(-.6, .6);
    */
    
  }

  public void RunFeeder(double speed) {

    FrontFeedMotor.set(-speed * 1.6);
    LeftFeedMotor.set(TalonFXControlMode.PercentOutput, speed);
    RightFeedMotor.set(TalonFXControlMode.PercentOutput, -speed);

  }
 
  public void RunAngle(double speed) {

    if (AngleEncoder.getPosition() < .68 && speed > 0) {
 
      System.out.println("Top limit hit, trying to go up " + speed);
      AngleMotor.set(0);

    } else if (AngleEncoder.getPosition() > .9 && speed < 0) {

      System.out.println("Bottom limit hit, trying to go down " + speed);
      AngleMotor.set(0);

    } else {

      System.out.println("Current Speed " + speed);
      AngleMotor.set(speed);

    }
 
  }

  public void RunAnglePID(double setPoint) {

    //AnglePID.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    System.out.println("Calculated Velocity " + AngleEncoder.getVelocity());

 
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("ArmEncodserValue", AngleEncoder.getPosition());

    // This method will be called once per scheduler run
  }
}
