package frc.robot;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class minMaxOutputConstants {
        /* Define max and min outputs */
        public static final int maxOutputRPM = 5700;
        public static final int kMaxOutput = 1;
        public static final int kMinOutput = -1;
    }

    public static final class ElevatorConstants {
        public static double kP = 0.0;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double kS = 0.0;
        public static double kV = 0.0;
        public static double kA = 0.0;

        public static double kAcceleration = 0.0;
        public static double kCruiseVelo = 0.0;
        public static double kJerk = 0.0;

        public static int topMotorID = 5;
        public static int bottomMotorID = 6;
    }

    public static final class IntakeConstants {
        public static double kP = 0.0;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double kS = 0.0;
        public static double kV = 0.0;

        public static int motor1ID = 0;
        public static int motor2ID = 0;
    }
    public static final class WristConstants {
        public static double kP = 0.0;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double kS = 0.0;
        public static double kV = 0.0;

        public static int motor1ID = 0;
        public static int motor2ID = 0;

        public static double kMaxSpeedMotionMagic = 0.0;
        public static double kMaxAccelerationMotionMagic = 0.0;
        public static double kMaxCurrentPerMotor = 0.0;
    }
}
