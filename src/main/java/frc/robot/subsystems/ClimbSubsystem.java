// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.armConstants;
import frc.robot.Constants.climbConstants;
import frc.robot.Constants.intakeConstants;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;


public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new Climb. */

  private SparkMax m_climbMotor;
  private SparkClosedLoopController m_PIDclimbMotor;
  private RelativeEncoder m_encoderClimbMotor;

  public ClimbSubsystem() {

    //Creats intake rotation motor
    m_climbMotor = new SparkMax(climbConstants.climbMotor, MotorType.kBrushless);
    m_PIDclimbMotor = m_climbMotor.getClosedLoopController();
    m_encoderClimbMotor = m_climbMotor.getEncoder();
    SparkMaxConfig m_climbMotorConfig = new SparkMaxConfig();

    m_climbMotorConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(0.1)
    .i(0.0)
    .d(0.0)
    .outputRange(-1,1);
    m_climbMotor.configure(m_climbMotorConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void pre_Climb_position(){
    m_PIDclimbMotor.setReference(-123,ControlType.kPosition);
  }

  public void climb_position(){
    m_PIDclimbMotor.setReference(-0, ControlType.kPosition);
  }
}
