// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import frc.lib.limelightOffset;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final PIDConstants kTranslationPID = new PIDConstants(1.0, 0.0, 0.0);
    public static final PIDConstants kRotationPID = new PIDConstants(1.0, 0.0, 0.0);
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 7;
    public static final int kRearLeftDrivingCanId = 9;
    public static final int kFrontRightDrivingCanId = 17 ;
    public static final int kRearRightDrivingCanId = 10;

    public static final int kFrontLeftTurningCanId = 6;
    public static final int kRearLeftTurningCanId = 8;
    public static final int kFrontRightTurningCanId = 19;
    public static final int kRearRightTurningCanId = 11;

    public static final boolean kGyroReversed = false;
  }

  public static final class limelightConstants {


    /**
     * PID constants for the autoalign
     */
     public static final double kPdrive = 0.1;
     public static final double kIdrive = 0;
     public static final double kDdrive = 0;

     public static final double kPstrafe = 0.08;
     public static final double kIstrafe = 0;
     public static final double kDstrafe = 0;

     public static final double kProtation = 0.04;
     public static final double kIrotation = 0;
     public static final double kDrotation = 0;


     public static final double X_REEF_ALIGNMENT_P = .5;
     public static final double Y_REEF_ALIGNMENT_P = .5;
     public static final double ROT_REEF_ALIGNMENT_P = 0.5;
  
     public static final double ROT_SETPOINT_REEF_ALIGNMENT = -8.0;
     public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 5.0;
     public static final double X_SETPOINT_REEF_ALIGNMENT = 3.3;
     public static final double X_TOLERANCE_REEF_ALIGNMENT = -0.2;
     public static final double Y_SETPOINT_REEF_ALIGNMENT = -1.6;
     public static final double Y_TOLERANCE_REEF_ALIGNMENT = .2;
  
     public static final double DONT_SEE_TAG_WAIT_TIME = 1;
     public static final double POSE_VALIDATION_TIME = 2;


        public static final class aprilTag{

            public static double driveOffset = 5.4;
            public static double strafeOffset = -1;
            public static double rotationOffset = 17;

            public static final limelightOffset offsets =  
        new limelightOffset(driveOffset, strafeOffset, rotationOffset);

    

        }
      }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class climbConstants{

    public static final int climbMotor = 16;
  }

  public static final class armConstants{

    public static final int armTelescope = 12;

  }

  public static final class intakeConstants{

    public static final int intakePivotMotor = 14;
    public static final int intakeMotorLeft = 13;
    public static final int intakeMotorRight = 15;

  }
}
