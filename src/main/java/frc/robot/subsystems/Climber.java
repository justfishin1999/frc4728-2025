// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.*;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Climber extends SubsystemBase {
  SparkMax m_climberMotor;
  SparkMaxConfig m_motorConfig;
  SparkClosedLoopController m_closedLoopController;
  RelativeEncoder m_encoder;
  double k_P, k_I, k_D, k_FF;
  int m_climberMotorID, k_maxAccel, k_maxVelo;

  /** Creates a new Climber. */
  public Climber() {
    k_P = Constants.ClimberConstants.kP;
    k_I = Constants.ClimberConstants.kI;
    k_D = Constants.ClimberConstants.kD;
    k_FF = Constants.ClimberConstants.kFF;
    m_climberMotorID = Constants.ClimberConstants.climberMotorID;
    k_maxAccel = Constants.ClimberConstants.kMaxMotionAcceleration;
    k_maxVelo = Constants.ClimberConstants.kMaxMotionVelocity;

    m_climberMotor = new SparkMax(m_climberMotorID, MotorType.kBrushless);

    m_motorConfig = new SparkMaxConfig();

    m_closedLoopController = m_climberMotor.getClosedLoopController();
    m_encoder = m_climberMotor.getEncoder();
    
    m_motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .p(k_P)
        .i(k_I)
        .d(k_D)
        .velocityFF(k_FF)
        .outputRange(Constants.minMaxOutputConstants.kMinOutput, Constants.minMaxOutputConstants.kMaxOutput, ClosedLoopSlot.kSlot1);

    m_motorConfig.closedLoop.maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .maxVelocity(k_maxVelo)
        .maxAcceleration(k_maxAccel)
        .allowedClosedLoopError(1);
    try{
      m_climberMotor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      System.out.println("Successfully configured Climber motor");
    }
    catch (Exception e1){
      System.err.println("Failed to apply Climber motor configurations: "+e1.getStackTrace());
      DriverStation.reportWarning("Failed to apply Climber motor configuration: "+e1.getStackTrace(),true);
    }
    

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Velocity",m_encoder.getVelocity());
    SmartDashboard.putNumber("Climber Position",m_encoder.getPosition());
    // This method will be called once per scheduler run
  }

  public void runClimber(double velocity){
    m_closedLoopController.setReference(velocity,ControlType.kVelocity);
  }

  public void resetEncoder(){
    m_encoder.setPosition(0.0);
  }

  public void stop(){
    m_closedLoopController.setReference(0,ControlType.kVelocity);
  }
}
