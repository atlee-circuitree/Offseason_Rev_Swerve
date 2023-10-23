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
  private double rollTolerance;

  private boolean initialMovement = true;
  
  private double horizontalSpeed;
  private double currentPitch;
  private double lastPitch = 0;
  private static final double PITCH_THRESHHOLD = .01;



  private String movementDirection = "Level";


  public AutoBalanceCommand(DriveSubsystem dt, double RollTolerance) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = dt;
    rollTolerance = RollTolerance;
   
    //addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Telemetry
    SmartDashboard.putNumber("Robot Straight Speed", horizontalSpeed);
    SmartDashboard.putString("Is robot level", movementDirection);
    SmartDashboard.putNumber("Pitch", currentPitch);

    currentPitch = drivetrain.getNavXRollOutput();

    //Speed PID calculations
    horizontalSpeed = Math.sin(Math.toRadians(currentPitch) * .58);

    /*if (lastPitch >= 0) {     //Code for positive Pitch
    
      if (lastPitch - currentPitch > PITCH_THRESHHOLD) {    //Ramp is falling
        horizontalSpeed = 0;
      }
      
    } else if (lastPitch < 0) {   //Code for negeative Pitch

      if (lastPitch - currentPitch < -PITCH_THRESHHOLD) {   //Ramp is falling
        horizontalSpeed = 0;
      }

    }

    lastPitch = currentPitch;*/

     //Speed clamps
    if (horizontalSpeed < -.3) {
      horizontalSpeed = -.3;
    } else if (horizontalSpeed > .3) {
      horizontalSpeed = .3;
    }

     if (currentPitch > rollTolerance || currentPitch < -rollTolerance) {

       initialMovement = false;

     } else {

      initialMovement = true;

     }

     //When NavX thinks tilted back, drive motors forward
     if (initialMovement = true) {

       drivetrain.drive(-.3, 0, 0, false, false);
       movementDirection = "Moving Backwards Initally";

     } else if (currentPitch > rollTolerance) {
        
       drivetrain.drive(horizontalSpeed, 0, 0, false, false);
       movementDirection = "Moving Forwards";
       
     //When NavX thinks tilted forward, drive motors backwards
     } else if (currentPitch < -rollTolerance) {

       drivetrain.drive(horizontalSpeed, 0, 0, false, false);
       movementDirection = "Moving Backwards";
  
     } else {

       drivetrain.drive(0, 0, 0, false, false);
       movementDirection = "Level";   
 
      } 

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