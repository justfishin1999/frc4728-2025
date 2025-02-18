package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase{
    TalonFX s_wristMotor;
    NeutralOut s_brake;
    MotionMagicVoltage s_request;

    public Wrist() {
        s_wristMotor = new TalonFX(Constants.WristConstants.motor1ID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();

        s_brake = new NeutralOut();

        config.Slot0.kP = Constants.WristConstants.kP;
        config.Slot0.kI = Constants.WristConstants.kI;
        config.Slot0.kD = Constants.WristConstants.kD;
        config.Slot0.kA = Constants.WristConstants.kA;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        config.MotionMagic.MotionMagicCruiseVelocity = Constants.WristConstants.kCruiseVelo;
        config.MotionMagic.MotionMagicAcceleration = Constants.WristConstants.kAcceleration;


        // enable stator current limit
        limitConfigs.StatorCurrentLimit = Constants.WristConstants.currentLimit; 
        limitConfigs.StatorCurrentLimitEnable = Constants.WristConstants.enableCurrentLimits;

        //try to apply configurations to motors, throw warning to driver station if it doesn't work
        try{
            s_wristMotor.getConfigurator().apply(config.Slot0);
            s_wristMotor.getConfigurator().apply(config.MotionMagic);
            s_wristMotor.getConfigurator().apply(limitConfigs);
            System.out.println("!!Successfully configured Wrist motor!!");

        } catch(Exception e1) {
            DriverStation.reportWarning("Failed to apply wrist motor configs",true);
            System.out.println("Failed to apply Wrist motor configs: "+e1.toString());
        }
        //set boolean false to true to invert motor direction

        s_request = new MotionMagicVoltage(0);

    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Wrist Position",s_wristMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Wrist Velocity",s_wristMotor.getVelocity().getValueAsDouble());
    }

    public void moveArm(double setpoint){
        s_wristMotor.setControl(s_request.withPosition(setpoint));
    }

    public void stop(){
        s_wristMotor.setControl(s_brake);
    }
}
