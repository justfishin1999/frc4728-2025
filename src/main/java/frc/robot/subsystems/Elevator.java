package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.revrobotics.spark.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class Elevator extends SubsystemBase{
    double s_kP, s_kI, s_kD, s_kS, s_kV, s_kGearRatio;
    int s_topMotorID, s_bottomMotorID;
    Follower s_follower;
    TalonFX s_elevator1, s_elevator2;

    private final MotionMagicTorqueCurrentFOC m_motionMagicTorqueCurrentFOC;


    public Elevator() {
        s_kP = Constants.ElevatorConstants.kP;
        s_kI = Constants.ElevatorConstants.kI;
        s_kD = Constants.ElevatorConstants.kD;
        s_kS = Constants.ElevatorConstants.kS;
        s_kV = Constants.ElevatorConstants.kV;
        s_kGearRatio = Constants.ElevatorConstants.kElevatorGearRatio;

        m_motionMagicTorqueCurrentFOC = new MotionMagicTorqueCurrentFOC(0);

        s_topMotorID = Constants.ElevatorConstants.topMotorID;
        s_bottomMotorID = Constants.ElevatorConstants.bottomMotorID; 

        s_elevator1 = new TalonFX(s_topMotorID);
        s_elevator2 = new TalonFX(s_bottomMotorID);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        config.Slot0.kP = s_kP;
        config.Slot0.kI = s_kI;
        config.Slot0.kD = s_kD;
        config.Slot0.kS = s_kS;
        config.Slot0.kV = s_kV;
        config.MotionMagic.MotionMagicAcceleration = Constants.ElevatorConstants.kMaxAccelerationMotionMagic;
        config.MotionMagic.MotionMagicCruiseVelocity = Constants.ElevatorConstants.kMaxSpeedMotionMagic;
        config.TorqueCurrent.PeakForwardTorqueCurrent = Constants.ElevatorConstants.kMaxCurrentPerMotor;
        config.TorqueCurrent.PeakReverseTorqueCurrent = Constants.ElevatorConstants.kMaxCurrentPerMotor;

        s_elevator1.getConfigurator().apply(config.Slot0);
        s_elevator2.getConfigurator().apply(config.Slot0);

        //set boolean false to true to invert motor direction
        s_follower = new Follower(s_topMotorID,false);

        s_elevator2.setControl(s_follower);
    }

    public void setElevatorSetpoint(double elevatorSetpoint){
        s_elevator1.setControl(m_motionMagicTorqueCurrentFOC.withPosition(elevatorSetpoint * s_kGearRatio));
    }

    public void periodic(){

    }

    public void stop(){
        //stop the motor
    }
}
