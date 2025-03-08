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
    m_encoderIntakePivotMotor = m_intakePivotMotor.getEncoder();
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
    .d(0.1)
    .outputRange(-1, 1);
    m_intakePivotMotor.configure(m_intakePivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //Configures Arm Pivot Motor
    m_armPivotMotorConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(0.2)
    .i(0)
    .d(0.05)
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

  public void intake_position(){
    m_PIDArmPivot.setReference(0,ControlType.kPosition);
    m_PIDIntakePivot.setReference(-3,ControlType.kPosition);
    m_PIDarmTelescope.setReference(0, ControlType.kPosition);
  };
 
  public final boolean arm_at_pos_neg15(){
    return ((m_encoderArmPivot.getPosition() > 0) && (m_encoderArmPivot.getPosition() < 0));
  };

  public final boolean intake_at_pos_neg4(){
    return (m_encoderArmPivot.getPosition() > -3.1) && (m_encoderArmPivot.getPosition() < -2.9);
  };

  public final boolean telescope_at_pos_0(){
    return (m_encoderArmTelescope.getPosition() > 0) && (m_encoderArmPivot.getPosition() < 0);
  };

  public final boolean armAtIntakePosition() {
    return (arm_at_pos_neg15()) && (intake_at_pos_neg4()) && (telescope_at_pos_0());
 };
 



  public void L1_position(){
    m_PIDArmPivot.setReference(0,ControlType.kPosition);
    m_PIDIntakePivot.setReference(0,ControlType.kPosition);
    m_PIDarmTelescope.setReference(2, ControlType.kPosition);
    };

  public final boolean arm_at_pos_01(){
   return ((m_encoderArmPivot.getPosition() > -2) && (m_encoderArmPivot.getPosition() < -2));
  };
  public final boolean intake_at_pos_01(){
     return (m_encoderIntakePivotMotor.getPosition() > 00) && (m_encoderIntakePivotMotor.getPosition() < 00);
  };
  public final boolean telescope_at_pos_01(){
    return (m_encoderArmTelescope.getPosition() > 2) && (m_encoderArmPivot.getPosition() < 2);
  };
  public final boolean armAtL1Position() {
     return (arm_at_pos_01()) && (intake_at_pos_neg4()) && (telescope_at_pos_01());
  };
  

  public void L2_position(){
    m_PIDArmPivot.setReference(0,ControlType.kPosition);
    m_PIDIntakePivot.setReference(-1.5,ControlType.kPosition);
    m_PIDarmTelescope.setReference(0, ControlType.kPosition);
    };

   // public final boolean arm_at_pos_neg1(){
      //return ((m_encoderArmPivot.getPosition() > 1) && (m_encoderArmPivot.getPosition() < 1));
     //};
     //public final boolean telescope_at_pos_6(){
       //return (m_encoderArmTelescope.getPosition() > 0) && (m_encoderArmPivot.getPosition() < 0);
   //  };
    public final boolean intake_at_pos_neg1p5(){
      return ((m_encoderIntakePivotMotor.getPosition() > -1.6) && (m_encoderIntakePivotMotor.getPosition() < 1.4));
    }
     public final boolean armAtL2Position() {
        return intake_at_pos_neg1p5()
        //(arm_at_pos_neg1()) 
        //&& (telescope_at_pos_6()
        ;
     };
   
  public void L3_position(){
    m_PIDArmPivot.setReference(0,ControlType.kPosition);
    m_PIDIntakePivot.setReference(0,ControlType.kPosition);
    m_PIDarmTelescope.setReference(261, ControlType.kPosition);
  };

  public final boolean arm_at_pos_10(){
    return ((m_encoderArmPivot.getPosition() > 3) && (m_encoderArmPivot.getPosition() < 3));
   };
   public final boolean intake_at_pos_00(){
      return (m_encoderIntakePivotMotor.getPosition() > 00) && (m_encoderIntakePivotMotor.getPosition() < 00);
   };
   public final boolean telescope_at_pos_222(){
     return (m_encoderArmTelescope.getPosition() > 261) && (m_encoderArmPivot.getPosition() < 261);
   };
   public final boolean armAtL3Position() {
      return (arm_at_pos_10()) && (intake_at_pos_00()) && (telescope_at_pos_222());
   };
 
  public void L4_position(){
    m_PIDArmPivot.setReference(7,ControlType.kPosition);
    m_PIDIntakePivot.setReference(-3,ControlType.kPosition);
    m_PIDarmTelescope.setReference(382, ControlType.kPosition);
  };

  
 public final boolean arm_at_pos_06(){
    return ((m_encoderArmPivot.getPosition() > 7) && (m_encoderArmPivot.getPosition() < 7));
   };
   public final boolean intake_at_pos_neg03(){
      return (m_encoderIntakePivotMotor.getPosition() > -3) && (m_encoderIntakePivotMotor.getPosition() < -3);
   };
   public final boolean telescope_at_pos_500(){
     return (m_encoderArmTelescope.getPosition() > 382) && (m_encoderArmPivot.getPosition() < 382);
   };
   public final boolean armAtL4Position() {
      return (arm_at_pos_06()) && (intake_at_pos_neg03()) && (telescope_at_pos_500());
   };

  public void keyRelease_position(){
    m_PIDArmPivot.setReference(0,ControlType.kPosition);
  };


  public final boolean arm_at_pos_05(){
    return ((m_encoderArmPivot.getPosition() > 4.9) && (m_encoderArmPivot.getPosition() < 5.1));
    };

   public final boolean armAtkeyReleasePosition() {
     return (arm_at_pos_05());
    };

 
  public void processor_position(){
    m_PIDArmPivot.setReference(0,ControlType.kPosition);
    m_PIDIntakePivot.setReference(0,ControlType.kPosition);
    m_PIDarmTelescope.setReference(0, ControlType.kPosition);
  };

/*   public final boolean arm_at_pos_06(){
    return ((m_encoderArmPivot.getPosition() > 00) && (m_encoderArmPivot.getPosition() < 00));
   };
   public final boolean intake_at_pos_06(){
      return (m_encoderIntakePivotMotor.getPosition() > 00) && (m_encoderIntakePivotMotor.getPosition() < 00);
   };
   public final boolean telescope_at_pos_06(){
     return (m_encoderArmTelescope.getPosition() > 00) && (m_encoderArmPivot.getPosition() < 00);
   };
   public final boolean armAtprocessorPosition() {
      return (arm_at_pos_00()) && (intake_at_pos_00()) && (telescope_at_pos_00());
   };
   */
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

