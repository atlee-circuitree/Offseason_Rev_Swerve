// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ChangeAngleCommand;
import frc.robot.commands.MaintainAngleCommand;
import frc.robot.commands.RotateModulesCommand;
import frc.robot.commands.RunAngleMotorCommand;
import frc.robot.commands.RunFeederCommand;
import frc.robot.commands.SetColorCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LightSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;
import java.util.Random;
import java.util.function.BooleanSupplier;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Other functions
  private Random rand = new Random();

  // Auto Selector
  private final SendableChooser<String> m_autoSelecter = new SendableChooser<>();

  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final LightSubsystem m_LightSubsystem = new LightSubsystem();
  private final FeederSubsystem m_FeederSubsystem = new FeederSubsystem();

  // The robot's commands
  private final SetColorCommand m_SetGreen = new SetColorCommand(m_LightSubsystem, .73);
  private final SetColorCommand m_SetRed = new SetColorCommand(m_LightSubsystem, .61);
  private final SetColorCommand m_SetBlue = new SetColorCommand(m_LightSubsystem, .87);
  private final SetColorCommand m_SetYellow = new SetColorCommand(m_LightSubsystem, .69);
  private final SetColorCommand m_SetForest = new SetColorCommand(m_LightSubsystem, -.99);

  // The driver's controller
  XboxController m_player1 = new XboxController(0);
  XboxController m_player2 = new XboxController(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    m_autoSelecter.setDefaultOption("Auto Balance", "Auto Balance");
    m_autoSelecter.addOption("Score And Back Up", "Score And Back Up");
    m_autoSelecter.addOption("Just Score", "Just Score");
    m_autoSelecter.addOption("Test", "Test");

    SmartDashboard.putData("Auto Selecter", m_autoSelecter);

    // Change the drive speed
    double speed = .8;
 
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband((-m_player1.getLeftY() * speed), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband((-m_player1.getLeftX() * speed), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_player1.getRightX() * speed), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

    m_FeederSubsystem.setDefaultCommand(new MaintainAngleCommand(m_FeederSubsystem));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    BooleanSupplier driver1LTSupplier = new BooleanSupplier() { @Override public boolean getAsBoolean() { if (m_player2.getLeftTriggerAxis() > 0.2) { return true; } else { return false; } } };
    Trigger driver2LT = new Trigger(driver1LTSupplier);

    BooleanSupplier driver1RTSupplier = new BooleanSupplier() { @Override public boolean getAsBoolean() { if (m_player2.getRightTriggerAxis() > 0.2) { return true; } else { return false; } } };
    Trigger driver2RT = new Trigger(driver1RTSupplier);

    BooleanSupplier driverDPadUpSupplier = new BooleanSupplier() {
        @Override
        public boolean getAsBoolean() {
            if(m_player1.getRawAxis(1) > .2){
              return true;
            }
            else{
              return false;
            }
          }
        };
    Trigger driverDPadUp = new Trigger(driverDPadUpSupplier);
 
    new JoystickButton(m_player1, XboxController.Button.kRightBumper.value)
        .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
    new JoystickButton(m_player1, XboxController.Button.kLeftBumper.value)
        .whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
    driver2LT
        .whileTrue(new RunFeederCommand(.1, m_FeederSubsystem));
    driver2RT
        .whileTrue(new RunFeederCommand(-.5, m_FeederSubsystem));
    new JoystickButton(m_player2, XboxController.Button.kA.value)
        .whileTrue(new ChangeAngleCommand(.8, m_FeederSubsystem));
    new JoystickButton(m_player2, XboxController.Button.kB.value)
        .whileTrue(new RunFeederCommand(-.85, m_FeederSubsystem));
    new JoystickButton(m_player2, XboxController.Button.kY.value)
        .whileTrue(new RunAngleMotorCommand(.6, m_FeederSubsystem));
    new JoystickButton(m_player2, XboxController.Button.kX.value)
        .whileTrue(new RunAngleMotorCommand(-.6, m_FeederSubsystem));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig configReversed = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);
    configReversed.setReversed(true);

    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    TrajectoryConfig configHalfSpeed = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond / 1.5, AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory GoBack = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(-1, 0), new Translation2d(-2, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(-5, 0, new Rotation2d(0)), 
        configReversed);

        Trajectory GoForward = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 0), new Translation2d(2, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(5, 0, new Rotation2d(0)), 
        config);

        Trajectory GoForwardHalfSpeed = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 0), new Translation2d(2, 0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(5, 0, new Rotation2d(0)), 
            config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand GoBackCommand = new SwerveControllerCommand(
        GoBack,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    SwerveControllerCommand GoBackCommandTo = new SwerveControllerCommand(
        GoBack,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,
    
            // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

        SwerveControllerCommand GoForwardCommand = new SwerveControllerCommand(
        GoForward,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,
        
        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

        SwerveControllerCommand GoForwardHalfSpeedCommand = new SwerveControllerCommand(
            GoForwardHalfSpeed,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
    
            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(GoBack.getInitialPose());

    // Autonomus Options
    SequentialCommandGroup ScoreAndDriveBack = new SequentialCommandGroup(
        new ChangeAngleCommand(.72, m_FeederSubsystem).withTimeout(.05),
        new MaintainAngleCommand(m_FeederSubsystem).withTimeout(1),
        new RunFeederCommand(-.65, m_FeederSubsystem).withTimeout(1),
        GoBackCommandTo.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false)));

    SequentialCommandGroup ScoreAndAutoBalance = new SequentialCommandGroup(
        new ChangeAngleCommand(.72, m_FeederSubsystem).withTimeout(.05),
        new MaintainAngleCommand(m_FeederSubsystem).withTimeout(1),
        new RunFeederCommand(-.85, m_FeederSubsystem).withTimeout(.3),
        GoBackCommand,
        GoForwardCommand.until(m_robotDrive.IsUnbalanced),
        GoForwardHalfSpeedCommand.until(m_robotDrive.IsBalanced));

    SequentialCommandGroup OnlyScore = new SequentialCommandGroup(
        new ChangeAngleCommand(.72, m_FeederSubsystem).withTimeout(.05),
        new MaintainAngleCommand(m_FeederSubsystem).withTimeout(1));

    SequentialCommandGroup Test = new SequentialCommandGroup();
 
    // Run path following command, then stop at the end.
    //return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
    
    if (m_autoSelecter.getSelected() == "Auto Balance") {

        return ScoreAndAutoBalance;

    } else if (m_autoSelecter.getSelected() == "Only Score") {

        return OnlyScore;

    } else if (m_autoSelecter.getSelected() == "Score And Back Up") {

        return ScoreAndDriveBack;

    } else {

        return Test;

    }

  }
}
