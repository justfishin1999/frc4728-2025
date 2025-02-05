package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    double s_kP, s_kI, s_kD, s_kV, s_kS, s_kFF;
    int s_manipulatorMotorID;
    SparkMax s_manipulatorMotor;
    SparkClosedLoopController s_closedLoopController;
    SparkMaxConfig s_motorConfig;
    RelativeEncoder s_encoder;


    public Intake() {
        s_kP = Constants.IntakeConstants.kP;
        s_kI = Constants.IntakeConstants.kI;
        s_kD = Constants.IntakeConstants.kD;
        s_kS = Constants.IntakeConstants.kS;
        s_kV = Constants.IntakeConstants.kV;
        s_kFF = Constants.IntakeConstants.kFF;

        s_manipulatorMotorID = Constants.IntakeConstants.motor1ID;

        s_manipulatorMotor = new SparkMax(s_manipulatorMotorID,MotorType.kBrushless);
        s_closedLoopController = s_manipulatorMotor.getClosedLoopController();

        s_motorConfig = new SparkMaxConfig();
        s_encoder = s_manipulatorMotor.getEncoder();

        s_motorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(s_kP)
            .i(s_kI)
            .d(s_kD)
            .velocityFF(s_kFF)
            .outputRange(Constants.minMaxOutputConstants.kMinOutput, Constants.minMaxOutputConstants.kMaxOutput);

        s_manipulatorMotor.configure(s_motorConfig,ResetMode.kResetSafeParameters,PersistMode.kNoPersistParameters);
            


    }

    public void periodic(){
        //constantly check and output the intake velocity to the dashboard
        SmartDashboard.putNumber("Intake Velocity",s_encoder.getVelocity());
    }

    public void runIntake_in(double velocity){
        //set the 'controller' for the manipulator to the desired velocity
        s_closedLoopController.setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    }



    public void stop(){
        //s_closedLoopController.setReference(0.0,ControlType.kCurrent,ClosedLoopSlot.kSlot1);
        s_manipulatorMotor.stopMotor();
    }
}
