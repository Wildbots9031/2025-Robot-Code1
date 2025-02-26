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
    .d(0.0)
    .outputRange(-1, 1);
    m_intakePivotMotor.configure(m_intakePivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //Configures Arm Pivot Motor
    m_armPivotMotorConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(0.1)
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
    m_PIDArmPivot.setReference(-2,ControlType.kPosition);
    m_PIDIntakePivot.setReference(-4,ControlType.kPosition);
    m_PIDarmTelescope.setReference(122, ControlType.kPosition);
  };
 
  public final boolean arm_at_pos_neg15(){
    return ((m_encoderArmPivot.getPosition() > -4) && (m_encoderArmPivot.getPosition() < -4));
  };

  public final boolean intake_at_pos_neg4(){
    return (m_encoderArmPivot.getPosition() > -4) && (m_encoderArmPivot.getPosition() < -4);
  };

  public final boolean telescope_at_pos_122(){
    return (m_encoderArmTelescope.getPosition() > 121) && (m_encoderArmPivot.getPosition() < 123);
  };

  public final boolean armAtIntakePosition() {
    return (arm_at_pos_neg15()) && (intake_at_pos_neg4()) && (telescope_at_pos_122());
 };
 



  public void L1_position(){
    m_PIDArmPivot.setReference(0,ControlType.kPosition);
    m_PIDIntakePivot.setReference(0,ControlType.kPosition);
    m_PIDarmTelescope.setReference(0, ControlType.kPosition);
    };

  public final boolean arm_at_pos_01(){
   return ((m_encoderArmPivot.getPosition() > 00) && (m_encoderArmPivot.getPosition() < 00));
  };
  public final boolean intake_at_pos_01(){
     return (m_encoderIntakePivotMotor.getPosition() > 00) && (m_encoderIntakePivotMotor.getPosition() < 00);
  };
  public final boolean telescope_at_pos_01(){
    return (m_encoderArmTelescope.getPosition() > 00) && (m_encoderArmPivot.getPosition() < 00);
  };
  public final boolean armAtL1Position() {
     return (arm_at_pos_01()) && (intake_at_pos_neg4()) && (telescope_at_pos_01());
  };
  

  public void L2_position(){
    m_PIDArmPivot.setReference(8,ControlType.kPosition);
    m_PIDIntakePivot.setReference(0,ControlType.kPosition);
    m_PIDarmTelescope.setReference(6, ControlType.kPosition);
    };

    public final boolean arm_at_pos_8(){
      return ((m_encoderArmPivot.getPosition() > 7.5) && (m_encoderArmPivot.getPosition() < 8.5));
     };
   //  public final boolean intake_at_pos_11(){
   //     return (m_encoderIntakePivotMotor.getPosition() > 00) && (m_encoderIntakePivotMotor.getPosition() < 00);
   //  };
     public final boolean telescope_at_pos_6(){
       return (m_encoderArmTelescope.getPosition() > 5) && (m_encoderArmPivot.getPosition() < 7);
     };
     public final boolean armAtL2Position() {
        return (arm_at_pos_8()) && (intake_at_pos_neg4()) && (telescope_at_pos_6());
     };
   
  public void L3_position(){
    m_PIDArmPivot.setReference(10,ControlType.kPosition);
    m_PIDIntakePivot.setReference(0,ControlType.kPosition);
    m_PIDarmTelescope.setReference(222, ControlType.kPosition);
  };

  public final boolean arm_at_pos_10(){
    return ((m_encoderArmPivot.getPosition() > 9.5) && (m_encoderArmPivot.getPosition() < 10.5));
   };
   public final boolean intake_at_pos_00(){
      return (m_encoderIntakePivotMotor.getPosition() > 00) && (m_encoderIntakePivotMotor.getPosition() < 00);
   };
   public final boolean telescope_at_pos_222(){
     return (m_encoderArmTelescope.getPosition() > 221) && (m_encoderArmPivot.getPosition() < 223);
   };
   public final boolean armAtL3Position() {
      return (arm_at_pos_10()) && (intake_at_pos_00()) && (telescope_at_pos_222());
   };
 
  public void L4_position(){
    m_PIDArmPivot.setReference(5,ControlType.kPosition);
    m_PIDIntakePivot.setReference(-4,ControlType.kPosition);
    m_PIDarmTelescope.setReference(533, ControlType.kPosition);
  };

  
 public final boolean arm_at_pos_05(){
    return ((m_encoderArmPivot.getPosition() > 5) && (m_encoderArmPivot.getPosition() < 5));
   };
   public final boolean intake_at_pos_neg04(){
      return (m_encoderIntakePivotMotor.getPosition() > -4) && (m_encoderIntakePivotMotor.getPosition() < -4);
   };
   public final boolean telescope_at_pos_533(){
     return (m_encoderArmTelescope.getPosition() > 532) && (m_encoderArmPivot.getPosition() < 534);
   };
   public final boolean armAtL4Position() {
      return (arm_at_pos_05()) && (intake_at_pos_neg04()) && (telescope_at_pos_533());
   };

  public void keyRelease_position(){
    m_PIDArmPivot.setReference(3,ControlType.kPosition);
    m_PIDIntakePivot.setReference(0,ControlType.kPosition);
    m_PIDarmTelescope.setReference(0, ControlType.kPosition);
  };


  public final boolean arm_at_pos_03(){
    return ((m_encoderArmPivot.getPosition() > 3) && (m_encoderArmPivot.getPosition() < 3));
    };
   public final boolean intake_at_pos_0(){
     return (m_encoderIntakePivotMotor.getPosition() > 00) && (m_encoderIntakePivotMotor.getPosition() < 00);
    };
   public final boolean telescope_at_pos_00(){
     return (m_encoderArmTelescope.getPosition() > 00) && (m_encoderArmPivot.getPosition() < 00);
    };
   public final boolean armAtkeyReleasePosition() {
     return (arm_at_pos_03()) && (intake_at_pos_0()) && (telescope_at_pos_00());
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
 
}
