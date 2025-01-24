// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.armConstants;
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


public class armSubsystem extends SubsystemBase {
  /** Creates a new armPivot. */

  private SparkMax m_intakePivotMotor;
  private SparkClosedLoopController m_PIDIntakePivot;
  private RelativeEncoder m_encoderIntakePivotMotor;


  private SparkMax m_armPivotMotor;
  private SparkClosedLoopController m_PIDArmPivot;
  private RelativeEncoder m_encoderArmPivot;

  private SparkMax m_armTelescopeMotor;
  private SparkClosedLoopController m_PIDarmTelescope;
  private RelativeEncoder m_encoderArmTelescope;


  public armSubsystem() {

    //Creates intake rotation motor
    m_intakePivotMotor = new SparkMax(intakeConstants.intakePivotMotor, MotorType.kBrushless);
    m_PIDIntakePivot = m_intakePivotMotor.getClosedLoopController();
    m_encoderIntakePivotMotor = m_armPivotMotor.getEncoder();
    SparkMaxConfig m_intakePivotMotorConfig = new SparkMaxConfig();

    //Creates arm pivot motor
    m_armPivotMotor = new SparkMax(armConstants.armPivotMotor, MotorType.kBrushless);
    m_PIDArmPivot = m_armPivotMotor.getClosedLoopController();
    m_encoderArmPivot = m_armPivotMotor.getEncoder();
    SparkMaxConfig m_armPivotMotorConfig = new SparkMaxConfig();
    
    //Creates Telescope Motor
    m_armTelescopeMotor = new SparkMax(armConstants.armTelescope, MotorType.kBrushless);
    m_PIDarmTelescope = m_armTelescopeMotor.getClosedLoopController();
    m_encoderArmTelescope = m_armTelescopeMotor.getEncoder();
    SparkMaxConfig m_armTelescopeMotorConfig = new SparkMaxConfig();


    //Configures Intake Pivot
    m_intakePivotMotorConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(0.1)
    .i(0.0)
    .d(0.0)
    .outputRange(-1, 1);
    m_intakePivotMotor.configure(m_intakePivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //Configures Arm Pivot Motor
    m_armPivotMotorConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(0.1)
    .i(0)
    .d(0)
    .outputRange(-1, 1);
    m_armPivotMotor.configure(m_armPivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    //Configures Telescope Motor
    m_armTelescopeMotorConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(0.1)
    .i(0.0)
    .d(0.0)
    .outputRange(-1, 1);
    m_armTelescopeMotor.configure(m_armTelescopeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public final boolean arm_at_pos_00(){
    return (m_encoderArmPivot.getPosition() > 00) && (m_encoderArmPivot.getPosition() < 00);
  };

  public void intake_position(){

    m_PIDArmPivot.setReference(0,ControlType.kPosition);
    m_PIDIntakePivot.setReference(0,ControlType.kPosition);
    m_PIDarmTelescope.setReference(0, ControlType.kPosition);
    };

}
