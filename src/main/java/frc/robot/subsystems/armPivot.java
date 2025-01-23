// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.armConstants;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.armConstants;


public class armPivot extends SubsystemBase {
  /** Creates a new armPivot. */

  private SparkMax m_armPivotMotor;
  private SparkClosedLoopController m_PIDArmPivot;
  private RelativeEncoder m_encoderArmPivot;

  public armPivot() {

    m_armPivotMotor = new SparkMax(armConstants.armPivotMotor, MotorType.kBrushless);
    m_PIDArmPivot = m_armPivotMotor.getClosedLoopController();
    m_encoderArmPivot = m_armPivotMotor.getEncoder();
    SparkMaxConfig m_armPivotMotorConfig = new SparkMaxConfig();


    m_armPivotMotorConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(0.1)
    .i(0)
    .d(0)
    .outputRange(0, 1);
    m_armPivotMotor.configure(m_armPivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
