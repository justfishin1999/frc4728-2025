package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class Wrist extends SubsystemBase{
    double s_kP, s_kI, s_kD, s_kV, s_kS;
    int s_topMotorID, s_bottomMotorID;
    TalonFX s_topMotor, s_bottomMotor;

    public Wrist() {
        s_kP = Constants.WristConstants.kP;
        s_kI = Constants.WristConstants.kI;
        s_kD = Constants.WristConstants.kD;
        s_kS = Constants.WristConstants.kS;
        s_kV = Constants.WristConstants.kV;

        s_topMotorID = Constants.WristConstants.motor1ID;
        s_bottomMotorID = Constants.WristConstants.motor2ID; 

        s_topMotor = new TalonFX(s_topMotorID);
        s_bottomMotor = new TalonFX(s_bottomMotorID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = s_kP;
        config.Slot0.kI = s_kI;
        config.Slot0.kD = s_kD;
        config.Slot0.kS = s_kS;
        config.Slot0.kV = s_kV;
        config.MotionMagic.MotionMagicAcceleration = Constants.WristConstants.kMaxAccelerationMotionMagic;
        config.MotionMagic.MotionMagicCruiseVelocity = Constants.WristConstants.kMaxSpeedMotionMagic;
        config.TorqueCurrent.PeakForwardTorqueCurrent = Constants.WristConstants.kMaxCurrentPerMotor;
        config.TorqueCurrent.PeakReverseTorqueCurrent = Constants.WristConstants.kMaxCurrentPerMotor;

        s_topMotor.getConfigurator().apply(config);
        s_bottomMotor.getConfigurator().apply(config);

    }

    public void periodic(){

    }



    public void stop(){
        //stop the motor
    }
}
