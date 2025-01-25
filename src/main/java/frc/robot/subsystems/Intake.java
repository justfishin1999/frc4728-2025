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

public class Intake extends SubsystemBase{
    double s_kP, s_kI, s_kD, s_kV, s_kS;
    int s_topMotorID, s_bottomMotorID;
    TalonFX s_topMotor, s_bottomMotor;

    public Intake() {
        s_kP = Constants.IntakeConstants.kP;
        s_kI = Constants.IntakeConstants.kI;
        s_kD = Constants.IntakeConstants.kD;
        s_kS = Constants.IntakeConstants.kS;
        s_kV = Constants.IntakeConstants.kV;

        s_topMotorID = Constants.IntakeConstants.motor1ID;
        s_bottomMotorID = Constants.IntakeConstants.motor2ID; 

        s_topMotor = new TalonFX(s_topMotorID);
        s_bottomMotor = new TalonFX(s_bottomMotorID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = s_kP;
        config.Slot0.kI = s_kI;
        config.Slot0.kD = s_kD;
        config.Slot0.kS = s_kS;
        config.Slot0.kV = s_kV;

        s_topMotor.getConfigurator().apply(config);
        s_bottomMotor.getConfigurator().apply(config);

    }

    public void periodic(){

    }



    public void stop(){
        //stop the motor
    }
}
