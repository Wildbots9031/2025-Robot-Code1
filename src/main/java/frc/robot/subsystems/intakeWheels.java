// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.intakeConstants;;


public class intakeWheels extends SubsystemBase {
  /** Creates a new intakeWheels. */

  private SparkMax m_intakeMotorLeft;
  private SparkMax m_intakeMotorRight;

  public intakeWheels() {
    m_intakeMotorLeft = new SparkMax(intakeConstants.intakeMotorLeft, MotorType.kBrushless);
    m_intakeMotorRight = new SparkMax(intakeConstants.intakeMotorRight, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
   
  public void intake_wheels_in(){
    m_intakeMotorLeft.set(1.0);
    m_intakeMotorRight.set(-1.0);
  };
  
  public void intake_wheels_out(){
    m_intakeMotorLeft.set(-1.0);
    m_intakeMotorRight.set(1.0);
  }

  public void intake_wheels_off(){
    m_intakeMotorLeft.set(0.0);
    m_intakeMotorRight.set(0.0);
  }

}
