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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;

//import the Constants
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

//import the Subsystems
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.intakeWheels;
import frc.robot.subsystems.ClimbSubsystem;

import frc.robot.commands.intake;
import frc.robot.commands.intakeWheelsIn;
import frc.robot.commands.intakeWheelsOff;
import frc.robot.commands.intakeWheelsOut;
import frc.robot.commands.keyRelease;
import frc.robot.commands.preClimbPosition;
import frc.robot.commands.L1;
import frc.robot.commands.L2;
import frc.robot.commands.L3;
import frc.robot.commands.L4;
import frc.robot.commands.climbPosition;
import frc.robot.commands.preClimbPosition;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final armSubsystem m_armSubsystem = new armSubsystem();
  private final intakeWheels m_intakeWheels = new intakeWheels();
  private final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
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
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));


        
    //Drive Controller buttons

        //Set A button on drive for intake wheels in
        JoystickButton aButtonDrive = new JoystickButton(m_driverController, XboxController.Button.kA.value);
            aButtonDrive.onTrue(new intakeWheelsIn(m_intakeWheels));

        //Set B Button for intake Wheels Out on drive
        JoystickButton bButtonDrive = new JoystickButton(m_driverController, XboxController.Button.kB.value);
            bButtonDrive.whileTrue(new intakeWheelsOut(m_intakeWheels));

        //Set X Button for Intake Wheels Off on drive
        JoystickButton xButtonDrive = new JoystickButton(m_driverController, XboxController.Button.kX.value);
            xButtonDrive.whileTrue(new intakeWheelsOff(m_intakeWheels));

        //Set Back Button For key release on drive
        JoystickButton backButtonDrive = new JoystickButton(m_driverController, XboxController.Button.kBack.value);
            backButtonDrive.onTrue(new keyRelease(m_armSubsystem));

        //Set Left Bumper for pre climb position on Drive
        JoystickButton leftBumperDrive = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);
            leftBumperDrive.onTrue(new preClimbPosition(m_ClimbSubsystem));

        //Set Right Bumper for climb position on Drive
        JoystickButton rightBumperDrive = new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);
            rightBumperDrive.onTrue(new climbPosition(m_ClimbSubsystem));

        //Set LeftStick Button for keyRelease on Drive
        //JoystickButton leftStickButtonDrive = new JoystickButton(m_driverController, XboxController.Button.kLeftStick.value);
          //  leftStickButtonDrive.onTrue(new keyRelease(m_armSubsystem));


    
    
    //Operator Controller Buttons

        //Set A button on Op for intake
        JoystickButton aButtonOp = new JoystickButton(m_operatorController, XboxController.Button.kA.value);
            aButtonOp.onTrue(new intake(m_armSubsystem));
   
       //Set B button on op for L3
        JoystickButton bButtonOp = new JoystickButton(m_operatorController, XboxController.Button.kB.value);
            bButtonOp.onTrue(new L3(m_armSubsystem));

        //Set X button on Op for L1
        JoystickButton xButtonOp = new JoystickButton(m_operatorController, XboxController.Button.kX.value);
            xButtonOp.onTrue(new L1(m_armSubsystem));

        //Set Y button on Op for L2
        JoystickButton yButtonOp = new JoystickButton(m_operatorController, XboxController.Button.kY.value);
            yButtonOp.onTrue(new L2(m_armSubsystem));
    
        //Set Left Bumper on Op for L4
        JoystickButton leftBumperButtonOp = new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value);
            leftBumperButtonOp.onTrue(new L4(m_armSubsystem));

        JoystickButton rightBumperButtonOp = new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value);
          //  rightBumperButtonOp.onTrue();
 
//AHHHHHHHHHHHHHHHHHHHHHHHHHHhh - hayley
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
}
