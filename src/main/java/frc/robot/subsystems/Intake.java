package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class Intake extends SubsystemBase{
    double s_kP, s_kI, s_kD, s_kS, s_kV, s_kA, s_Acceleration, s_CruiseVelo, s_Jerk, s_motionMagicA, s_motionMagicV;
    int s_motorID;
    TalonFX s_IntakeMotor;
    NeutralOut s_brake;
    VelocityVoltage m_velocityVoltage;

    public Intake() {
        s_kP = Constants.IntakeConstants.kP;
        s_kI = Constants.IntakeConstants.kI;
        s_kD = Constants.IntakeConstants.kD;
        s_kS = Constants.IntakeConstants.kS;
        s_kV = Constants.IntakeConstants.kV;
        s_kA = Constants.IntakeConstants.kA;

        s_motorID = Constants.IntakeConstants.motor1ID;

        m_velocityVoltage = new VelocityVoltage(0).withSlot(0);

        s_IntakeMotor = new TalonFX(s_motorID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = s_kP;
        config.Slot0.kI = s_kI;
        config.Slot0.kD = s_kD;
        config.Slot0.kS = s_kS;
        config.Slot0.kV = s_kV;
        config.Slot0.kA = s_kA;

        try{
            s_IntakeMotor.getConfigurator().apply(config);
        }
        catch(Exception e1){
            DriverStation.reportWarning(getName(),e1.getStackTrace());
            System.out.println("Failed to apply Intake motor configs: "+e1.getStackTrace());
        }

    }

    public void periodic(){
        SmartDashboard.putNumber("Intake Velocity",s_IntakeMotor.getVelocity().getValueAsDouble());
    }

    public void actuateArm(double setpoint){
        s_IntakeMotor.setControl(m_velocityVoltage.withVelocity(setpoint));
    }

    public void stop(){
        s_IntakeMotor.setControl(s_brake);
    }
}
