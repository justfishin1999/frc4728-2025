package frc.robot.commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAlign_Left extends Command {

    private CommandSwerveDrivetrain drivetrain;
    private SwerveRequest.RobotCentric limeDrive;
    private double alignmentSpeed;
    private double tx;
    
        public AutoAlign_Left(CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric limeDrive, double alignmentSpeed) {
            this.drivetrain = drivetrain;
            this.limeDrive = limeDrive;  // Now we pass in limeDrive to the command
            this.alignmentSpeed = alignmentSpeed;
            addRequirements(drivetrain);  // Ensure the drivetrain is required for this command
        }
    
        @Override
        public void initialize() {
            DriverStation.reportWarning("caught auto loop",false);
            // Initialize command, can be used for any pre-execution logic (e.g., turning on pipeline)
        }
    
        @Override
        public void execute() {
            // Get the horizontal offset (tx) from Limelight
            tx = LimelightHelpers.getTX("limelight");

        // Use limeDrive (already set up in RobotContainer) to adjust the robot's rotation
        drivetrain.applyRequest(() -> limeDrive.withVelocityY(-tx * alignmentSpeed));
        }

    @Override
    public boolean isFinished() {
        // The command finishes when tx is sufficiently close to zero (i.e., the robot is aligned)
        if(tx<1&&tx>0){
            DriverStation.reportWarning("ended auto command",false);
            return true;
        } else{
            return false;
        }  // You can adjust this threshold as needed
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot after alignment
        drivetrain.applyRequest(() -> limeDrive.withVelocityY(0));  // Stop robot motion
    }
}
