// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
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

  private SparkMax m_intakePivotMotor;
  private SparkClosedLoopController m_PIDIntakePivot;
  private RelativeEncoder m_encoderIntakePivotMotor;

  private SparkMax m_armTelescopeMotor;
  private SparkClosedLoopController m_PIDarmTelescope;
  private RelativeEncoder m_encoderArmTelescope;


  public armSubsystem() {


    //Creates intake rotation motor
    m_intakePivotMotor = new SparkMax(intakeConstants.intakePivotMotor, MotorType.kBrushless);
    m_PIDIntakePivot = m_intakePivotMotor.getClosedLoopController();
    m_encoderIntakePivotMotor = m_intakePivotMotor.getEncoder();
    SparkMaxConfig m_intakePivotMotorConfig = new SparkMaxConfig();
    
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
    .d(0.15)
    .outputRange(-1, 1);
    m_intakePivotMotor.configure(m_intakePivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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

  public void intake_position(){
    m_PIDIntakePivot.setReference(-2,ControlType.kPosition);
    m_PIDarmTelescope.setReference(0, ControlType.kPosition);
      
  };
 
  public final boolean intake_at_pos_neg3(){
    return (m_encoderIntakePivotMotor.getPosition() > -3) && (m_encoderIntakePivotMotor.getPosition() > -3);
  }

  public final boolean telescope_at_pos_0(){
    return (m_encoderArmTelescope.getPosition() > 0) && (m_encoderArmTelescope.getPosition() < 0);
  };

  public final boolean armAtIntakePosition() {
    return (intake_at_pos_neg3()) && (telescope_at_pos_0());
 };
 



  public void L1_position(){
    m_PIDIntakePivot.setReference(0,ControlType.kPosition);
    m_PIDarmTelescope.setReference(2, ControlType.kPosition);
    };

  public final boolean intake_at_L1_pos(){
     return (m_encoderIntakePivotMotor.getPosition() > 00) && (m_encoderIntakePivotMotor.getPosition() < 00);
  };
  public final boolean telescope_at_pos_01(){
    return (m_encoderArmTelescope.getPosition() > 2) && (m_encoderArmTelescope.getPosition() < 2);
  };
  public final boolean armAtL1Position() {
     return (intake_at_L1_pos()) && (telescope_at_pos_01());
  };
  

  public void L2_position(){
    m_PIDIntakePivot.setReference(0,ControlType.kPosition);
    m_PIDarmTelescope.setReference(0, ControlType.kPosition);
    };

    public final boolean intake_at_pos_neg1p5(){
      return ((m_encoderIntakePivotMotor.getPosition() >= 0) && (m_encoderIntakePivotMotor.getPosition() < 0 ));
    }
     public final boolean armAtL2Position() {
        return intake_at_pos_neg1p5();
     };
   
  public void L3_position(){
    m_PIDIntakePivot.setReference(0,ControlType.kPosition);
    m_PIDarmTelescope.setReference(305, ControlType.kPosition);
  };

   public final boolean intake_at_pos_00(){
      return (m_encoderIntakePivotMotor.getPosition() > 00) && (m_encoderIntakePivotMotor.getPosition() < 00);
   };
   public final boolean telescope_at_pos_275(){
     return (m_encoderArmTelescope.getPosition() > 305 && (m_encoderArmTelescope.getPosition() < 305));
   };
   public final boolean armAtL3Position() {
      return (intake_at_pos_00()) && (telescope_at_pos_275());
   };
 
  public void L4_position(){
    m_PIDIntakePivot.setReference(-3,ControlType.kPosition);
    m_PIDarmTelescope.setReference(382, ControlType.kPosition);
  };

   public final boolean intake_at_pos_neg03(){
      return (m_encoderIntakePivotMotor.getPosition() > -3) && (m_encoderIntakePivotMotor.getPosition() < -3);
   };
   public final boolean telescope_at_pos_500(){
     return (m_encoderArmTelescope.getPosition() > 382) && (m_encoderArmTelescope.getPosition() < 382);
   };
   public final boolean armAtL4Position() {
      return (intake_at_pos_neg03()) && (telescope_at_pos_500());
   };
 
  public void processor_position(){
    m_PIDIntakePivot.setReference(0,ControlType.kPosition);
    m_PIDarmTelescope.setReference(0, ControlType.kPosition);
  };

  public void raise_intake(){
    double currentPosition = m_encoderIntakePivotMotor.getPosition();
    double armUp =  currentPosition - .3;

    m_PIDIntakePivot.setReference(armUp, ControlType.kPosition);
   // m_PIDIntakePivot.setReference(0, ControlType.kPosition);
  }

  public void lower_intake(){
    double currentPosition = m_encoderIntakePivotMotor.getPosition();
    double armDown =  currentPosition + .3;

    m_PIDIntakePivot.setReference(armDown, ControlType.kPosition);
   // m_PIDIntakePivot.setReference(0, ControlType.kPosition);

  
  
}
public final boolean intakeDownTrue() {
  double currentPosition = m_encoderIntakePivotMotor.getPosition();
  double armDown =  currentPosition + .3;
  return (m_encoderIntakePivotMotor.getPosition() >= armDown);

}
public final boolean intakeUpTrue() {
  double currentPosition = m_encoderIntakePivotMotor.getPosition();
  double armUp =  currentPosition - .3;
  return (m_encoderIntakePivotMotor.getPosition() >= armUp);

}

}

