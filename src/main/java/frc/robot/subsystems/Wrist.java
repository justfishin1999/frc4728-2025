package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class Wrist extends SubsystemBase{
    double s_kP, s_kI, s_kD, s_kS, s_kV, s_kA, s_Acceleration, s_CruiseVelo, s_Jerk, s_motionMagicA, s_motionMagicV;
    int s_motorID;
    TalonFX s_wristMotor;
    NeutralOut s_brake;
    MotionMagicVoltage s_request;

    public Wrist() {
        s_kP = Constants.WristConstants.kP;
        s_kI = Constants.WristConstants.kI;
        s_kD = Constants.WristConstants.kD;
        s_kS = Constants.WristConstants.kS;
        s_kV = Constants.WristConstants.kV;
        s_kA = Constants.WristConstants.kA;

        s_motorID = Constants.WristConstants.motor1ID;

        s_wristMotor = new TalonFX(s_motorID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = s_kP;
        config.Slot0.kI = s_kI;
        config.Slot0.kD = s_kD;
        config.Slot0.kS = s_kS;
        config.Slot0.kV = s_kV;
        config.Slot0.kA = s_kA;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
        
        config.MotionMagic.MotionMagicCruiseVelocity = s_CruiseVelo;
        config.MotionMagic.MotionMagicAcceleration = s_Acceleration;
        config.MotionMagic.MotionMagicJerk = s_Jerk;
        config.MotionMagic.MotionMagic_kA = s_motionMagicA;
        config.MotionMagic.MotionMagic_kV = s_motionMagicV;

        try{
            s_wristMotor.getConfigurator().apply(config);
        }
        catch(Exception e1){
            DriverStation.reportWarning(getName(),e1.getStackTrace());
            System.out.println("Failed to apply wrist motor configs: "+e1.getStackTrace());
        }
        s_request = new MotionMagicVoltage(0);

    }

    public void periodic(){
        SmartDashboard.putNumber("Wrist Position",s_wristMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Wrist Velocity",s_wristMotor.getVelocity().getValueAsDouble());
    }

    public void actuateArm(double setpoint){
        s_wristMotor.setControl(s_request.withPosition(setpoint));
    }

    public void stop(){
        s_wristMotor.setControl(s_brake);
    }
}
