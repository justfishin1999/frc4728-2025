package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.revrobotics.spark.*;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class Elevator extends SubsystemBase{
    double s_kP, s_kI, s_kD, s_kS, s_kV, s_kG, s_Acceleration, s_CruiseVelo, s_Jerk;
    int s_topMotorID, s_bottomMotorID;
    Follower s_follower;
    TalonFX s_elevator1, s_elevator2;
    PositionVoltage s_positionVoltage;
    NeutralOut s_brake;


    public Elevator() {
        s_kP = Constants.ElevatorConstants.kP;
        s_kI = Constants.ElevatorConstants.kI;
        s_kD = Constants.ElevatorConstants.kD;
        s_kG = Constants.ElevatorConstants.kG;
        s_Acceleration = Constants.ElevatorConstants.kAcceleration;
        s_CruiseVelo = Constants.ElevatorConstants.kCruiseVelo;
        s_Jerk = Constants.ElevatorConstants.kJerk;
        s_topMotorID = Constants.ElevatorConstants.topMotorID;
        s_bottomMotorID = Constants.ElevatorConstants.bottomMotorID; 

        s_elevator1 = new TalonFX(s_topMotorID);
        s_elevator2 = new TalonFX(s_bottomMotorID);

        TalonFXConfiguration config = new TalonFXConfiguration();

        s_positionVoltage = new PositionVoltage(0).withSlot(0);
        s_brake = new NeutralOut();

        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        config.Slot0.kP = s_kP;
        config.Slot0.kI = s_kI;
        config.Slot0.kD = s_kD;
        config.Slot0.kG = s_kG;

        config.MotionMagic.MotionMagicCruiseVelocity = s_CruiseVelo;
        config.MotionMagic.MotionMagicAcceleration = s_Acceleration;
        config.MotionMagic.MotionMagicJerk = s_Jerk;

        config.Voltage.withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8));
        //try to apply configurations to motors, throw warning to driver station if it doesn't work
        try{
            s_elevator1.getConfigurator().apply(config.Slot0);
            s_elevator2.getConfigurator().apply(config.Slot0);
        } catch(Exception e1) {
            DriverStation.reportWarning(getName(), e1.getStackTrace());
            System.out.println("Failed to apply elevator motor configs: "+e1.getStackTrace());
        }
        //set boolean false to true to invert motor direction
        s_follower = new Follower(s_topMotorID,true);
        s_elevator2.setControl(s_follower);
    }

    public void setElevatorSetpoint(double elevatorSetpoint){
        //s_elevator2.setControl(s_positionVoltage.withPosition(elevatorSetpoint));
        s_elevator2.setPosition(elevatorSetpoint);
    }

    public void periodic(){
        s_elevator2.getRotorPosition();
    }

    public void stop(){
        s_elevator2.setControl(s_brake);
    }
}
